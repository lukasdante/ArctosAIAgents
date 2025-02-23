from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool
import rclpy

import whisper
import base64
import io
import soundfile as sf
import torch
import librosa

class Writer(Node):
	def __init__(self):
		super().__init__('writer')

		self.declare_parameter('language', 'en-US', ParameterDescriptor(description='Language of the transcription output.'))
		self.declare_parameter('sample_rate', 16000, ParameterDescriptor(description='Sample rate of the audio recording.'))
		self.declare_parameter('encoding_format', 'LINEAR16', ParameterDescriptor(description='Encoding format of the audio recording.'))
		self.declare_parameter('local_model', 'base.en', ParameterDescriptor(description='OpenAI Whisper model type.'))
		self.declare_parameter('inference', 'local', ParameterDescriptor(description="The inference method, can be 'local' or 'cloud'."))

		self.language = self.get_parameter('language').get_parameter_value().string_value
		self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
		self.encoding_format = self.get_parameter('encoding_format').get_parameter_value().string_value
		self.inference = self.get_parameter('inference').get_parameter_value().string_value
		self.local_model = whisper.load_model(self.get_parameter('local_model').get_parameter_value().string_value)

		self.publisher = self.create_publisher(String, 'conversation/transcript', 10)
		# self.actor_publisher = self.create_publisher(String, 'actor/conversation/transcript', 10)
		# self.reset_publisher = self.create_publisher(Bool, 'conversation/reset', 10)
		self.subscriber = self.create_subscription(String, 'conversation/recording', self.write, 10)

		self.get_logger().info("Writer node initialized.")

	def write(self, msg):
		if self.inference == 'local':
			self.transcribe_local(msg)

		if self.inference == 'cloud':
			self.transcribe_cloud(msg)

	def decode_audio_from_string(self, msg: str):
		audio_bytes = base64.b64decode(msg)
		audio_buffer = io.BytesIO(audio_bytes)
		audio_data, sample_rate = sf.read(audio_buffer)

		return audio_data, sample_rate

	def transcribe_local(self, msg: String):
		# local inference needs an audio file
		audio_data, sample_rate = self.decode_audio_from_string(msg.data)

		# Convert to 16 kHz (Whisper expects 16kHz mono audio)
		if self.sample_rate != 16000:
			audio_data = librosa.resample(audio_data, orig_sr=sample_rate, target_sr=16000)

		# Transcribe using Whisper
		audio_tensor = torch.tensor(audio_data).float()  # Convert to tensor
		result = self.local_model.transcribe(audio_tensor)

		# Print the transcribed text
		transcription = String()
		transcription.data = result["text"]
		self.publisher.publish(transcription)

		self.get_logger().info(f'{result["text"]}')

	def transcribe_cloud():
		pass

def main(args=None):
	try:
		rclpy.init(args=args)
		lone_writer = Writer()
		rclpy.spin(lone_writer)
	except KeyboardInterrupt:
		lone_writer.destroy_node()
		pass

if __name__ == '__main__':
	main()