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
import requests
import os

class Writer(Node):
	def __init__(self):
		super().__init__('writer')

		# Declare ROS 2 parameters
		self.declare_parameter('language', 'en-US', ParameterDescriptor(description='Language of the transcription output.'))
		self.declare_parameter('sample_rate', 16000, ParameterDescriptor(description='Sample rate of the audio recording.'))
		self.declare_parameter('encoding_format', 'LINEAR16', ParameterDescriptor(description='Encoding format of the audio recording.'))
		self.declare_parameter('local_model', 'base.en', ParameterDescriptor(description='OpenAI Whisper model type.'))
		self.declare_parameter('inference', 'local', ParameterDescriptor(description="The inference method, can be 'local' or 'cloud'."))

		# Declare class properties
		self.language = self.get_parameter('language').get_parameter_value().string_value
		self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
		self.encoding_format = self.get_parameter('encoding_format').get_parameter_value().string_value
		self.inference = self.get_parameter('inference').get_parameter_value().string_value
		self.local_model = whisper.load_model(self.get_parameter('local_model').get_parameter_value().string_value)

		# Declare cloud credentials
		self.api_key = os.getenv('STT_API_KEY')
		self.api_endpoint = f'https://speech.googleapis.com/v1/speech:recognize?key={self.api_key}'

		# Declare class publishers and subscribers
		self.publisher = self.create_publisher(String, 'conversation/transcript', 10)
		self.subscriber = self.create_subscription(String, 'conversation/recording', self.write, 10)

		self.get_logger().info("Writer node initialized.")

	def write(self, msg):
		""" Transcribes an audio from base64-encoded string to string. """

		# Runs local inference
		if self.inference == 'local':
			self.transcribe_local(msg)

		# Runs cloud inference
		if self.inference == 'cloud':
			self.transcribe_cloud(msg)

	def decode_audio_from_string(self, msg: str):
		""" Decodes audio from base64 string. """

		# Decode base64 string to audio bytes
		audio_bytes = base64.b64decode(msg)
		audio_buffer = io.BytesIO(audio_bytes) # Save audio to a buffer
		audio_data, sample_rate = sf.read(audio_buffer) # Read audio data

		return audio_data, sample_rate
	
	def publish_transcript(self, transcript):
		""" Publish the transcribed text to conversation/transcript. """
		transcription = String()
		transcription.data = transcript
		self.publisher.publish(transcription)
		self.get_logger().info(f'{transcript}')


	def transcribe_local(self, msg: String):
		""" Local speech-to-text inference. """

		# Decode base64 string to audio bytes
		audio_data, sample_rate = self.decode_audio_from_string(msg.data)

		# Convert to 16 kHz (Whisper expects 16kHz mono audio)
		if self.sample_rate != 16000:
			audio_data = librosa.resample(audio_data, orig_sr=sample_rate, target_sr=16000)

		# Transcribe using Whisper
		audio_tensor = torch.tensor(audio_data).float()  # Convert to tensor
		result = self.local_model.transcribe(audio_tensor)

		# Publish the transcription
		if result["text"]:
			self.publish_transcript(result["text"])
		else:
			self.get_logger().info("No transcription found, resetting conversation.")

	def transcribe_cloud(self, msg: String):
		""" Google Cloud speech-to-text inference. """

		# Convert ROS2 String message to raw string content
		audio_content = msg.data  # Extract raw audio content from the ROS2 message

		# Configure the request
		headers = {'Content-Type': 'application/json'}
		body = {
			'config': {
				'encoding': self.encoding_format,
				'sampleRateHertz': self.sample_rate,
				'languageCode': self.language
			},
			'audio': {
				'content': audio_content
			}
		}

		# Send the request
		response = requests.post(self.api_endpoint, headers=headers, json=body)

		# Check if the request is successful
		if response and response.status_code == 200:
			result = response.json()

			# Get the transcription from results
			if 'results' in result:
				for res in result['results']:
					transcription = res['alternatives'][0]['transcript']
					
					self.publish_transcript(transcription)
			
			else:
				self.get_logger().info("No transcription found, resetting conversation.")

		else:
			self.get_logger().error(f"Writer request error {response.status_code if response else 'unknown'}.")

def main(args=None):
	try:
		rclpy.init(args=args)
		lone_writer = Writer() # instantiate Writer Node
		rclpy.spin(lone_writer)
	except KeyboardInterrupt:
		lone_writer.destroy_node()
		pass

if __name__ == '__main__':
	main()