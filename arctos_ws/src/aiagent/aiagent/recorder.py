from utils.utils.nodes import BaseNode
import rclpy
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import ParameterDescriptor

import pyaudio
import wave
import numpy
import io
import base64
from ctypes import *


class Recorder(BaseNode):
	def __init__(self):
		super().__init__('recorder')
		
		try:
			self.declare_parameter('chunk_size', 1024, ParameterDescriptor(description='Chunk size of the audio recording.'))
			self.declare_parameter('channels', 1, ParameterDescriptor(description='Number of channels of the audio recording.'))
			self.declare_parameter('sample_rate', 16000, ParameterDescriptor(description='Sample rate of the audio recording.'))
			self.declare_parameter('threshold', 1000, ParameterDescriptor(description='Silence volume threshold of audio recording for automatic termination.'))
			# MAKE SURE THE SILENCE_LIMIT IS A FLOAT
			self.declare_parameter('silence_limit', float(2.5), ParameterDescriptor(description='Silence length until audio recording terminates.'))

			self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
			self.channels = self.get_parameter('channels').get_parameter_value().integer_value
			self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
			self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
			self.silence_limit = self.get_parameter('silence_limit').get_parameter_value().double_value

			self.publisher = self.create_publisher(String, 'conversation/request_audio', 10)
			self.subscription = self.create_subscription(Bool,'conversation/reset', self.record, 10)
			self.parameter_subscriber = self.create_subscription(String, 'conversation/parameters', self.change_property, 10)
			self.volume_pub = self.create_publisher(String, 'conversation/volume', 10)


			self.get_logger().info("Recorder initialized.")
		except Exception as e:
			self.get_logger().error(f"Unable to initialize recorder: {e}")

	def change_property(self, msg):
		try:
			self.change_parameter(msg)
			self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
			self.silence_limit = self.get_parameter('silence_limit').get_parameter_value().double_value
			
		except:
			return


	def open_audio_stream(self):
		"""Open the audio stream."""
		audio = pyaudio.PyAudio()

		stream = audio.open(format=pyaudio.paInt16,
							channels=self.channels,
							rate=self.sample_rate,
							input=True,
							frames_per_buffer=self.chunk_size)
		
		self.get_logger().info('The audio stream has been opened.')
		
		return audio, stream
	
	def get_volume(self, data):
		""" Obtains the mean absolute value of the current audio. """

		if not data:
			return 0
		audio_data = numpy.frombuffer(data, dtype=numpy.int16)
		return numpy.abs(audio_data).mean()

	def record_audio_stream(self, stream: pyaudio.Stream):
		"""Record audio stream, returns a list of frames of the audio chunks."""
		frames = []
		silent_chunks = 0
		initial_time = self.get_clock().now().nanoseconds
		while (True):
			data = stream.read(self.chunk_size)

			frames.append(data)
			
			# Calculate the volume
			volume = self.get_volume(data)

			# For every 200ms display the volume
			volume_msg = String()
			volume_msg.data = f'Current volume: {int(volume)} -> {self.threshold}'
			self.get_logger().info(f'Current volume: {int(volume)} -> {self.threshold}')
			self.volume_pub.publish(volume_msg)
			
			# Record silent chunks and stop recording once limit is reached
			if self.is_silent(volume, initial_time, silent_chunks):
				break
			
		return frames

	def is_silent(self, volume, initial_time, silent_chunks) -> bool:
		"""Determines if the audio is silent to terminate recording, returns True if silent."""
		if volume < self.threshold:
			silent_chunks += 1
		else:
			silent_chunks = 0
		
		if silent_chunks >= int(self.silence_limit * self.sample_rate / self.chunk_size + 0.05):
			if (self.get_clock().now().nanoseconds - initial_time) > ((self.silence_limit + 0.1) * 1e9):
				self.get_logger().info("Silence detected. Stopping recording...")
				return True
			
		return False

	def save_to_buffer(self, frames, audio: pyaudio.PyAudio) -> io.BytesIO:
		"""Save to a WAV buffer, returns an io.BytesIO() object."""
		wav_buffer = io.BytesIO()

		# Write the WAV data into the buffer
		with wave.open(wav_buffer, 'wb') as wf:
			wf.setnchannels(self.channels)
			wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
			wf.setframerate(self.sample_rate)
			wf.writeframes(b''.join(frames))

		return wav_buffer
	
	def encode_wav_to_string(self, wav_buffer: io.BytesIO):
		# Get the WAV data from the buffer
		wav_buffer.seek(0)
		audio_content = wav_buffer.read()
		encoded_audio = base64.b64encode(audio_content).decode('utf-8')

		return encoded_audio

	def record(self, msg: Bool):
		""" Record audio until silence is detected. """
		try:
			audio, stream = self.open_audio_stream()    
			self.get_logger().info("Recording started. Speak into the microphone...")
		except Exception as e:
			self.get_logger().error(f"Failed to open audio stream: {e}.")
			return

		# Read and store the audio stream until silence is detected
		frames = self.record_audio_stream(stream)
				
		# Stop and close the stream
		stream.stop_stream()
		stream.close()
		audio.terminate()

		# Save recording to buffer
		wav_buffer: io.BytesIO = self.save_to_buffer(frames, audio)

		# Publish message to topic `conversation/request_audio`
		msg = String()
		msg.data = self.encode_wav_to_string(wav_buffer)
		self.publisher.publish(msg)

def clean_asla():
	"""Handle ASLA errors for cleaner output."""
	ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
	def py_error_handler(filename, line, function, err, fmt):
		pass
	c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
	asound = cdll.LoadLibrary('libasound.so')
	asound.snd_lib_error_set_handler(c_error_handler)

def main(args=None):
	clean_asla()
	try:
		rclpy.init(args=args)
		lone_recorder = Recorder()
		rclpy.spin(lone_recorder)
	except KeyboardInterrupt:
		lone_recorder.destroy_node()
		pass

if __name__ == '__main__':
	main()