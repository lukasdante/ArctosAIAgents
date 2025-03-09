import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool
from utils.nodes import BaseNode
from interfaces.srv import TalkString

from kokoro import KPipeline
import sounddevice as sd
from dotenv import load_dotenv
import torch
import json
import requests
import io
import json
import time
import base64
import os
import wave
import pyaudio

from google.auth import jwt as google_jwt
from google.auth import crypt
from google.oauth2 import service_account


class Talker(BaseNode):
	def __init__(self):
		super().__init__('talker')

		# Declare node parameters
		self.declare_parameter('language', 'en-US', ParameterDescriptor(description='Language of the transcription output.'))
		self.declare_parameter('gender', 'FEMALE', ParameterDescriptor(description='Gender of the text-to-speech voice agent.'))
		self.declare_parameter('accent', 'en-US-Neural2-C', ParameterDescriptor(description='Accent or voice type of the text-to-speech voice agent.'))
		self.declare_parameter('encoding_format', 'LINEAR16', ParameterDescriptor(description='Encoding format of the audio recording.'))
		self.declare_parameter('inference', 'local', ParameterDescriptor(description="The inference method, can be 'local' or 'cloud'."))
		self.declare_parameter('sample_rate', 24000, ParameterDescriptor(description="Sample rate of audio playback."))

		# Set node parameters
		self.language = self.get_parameter('language').get_parameter_value().string_value
		self.gender = self.get_parameter('gender').get_parameter_value().string_value
		self.accent = self.get_parameter('accent').get_parameter_value().string_value
		self.encoding_format = self.get_parameter('encoding_format').get_parameter_value().string_value
		self.inference = self.get_parameter('inference').get_parameter_value().string_value
		self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value

		# set cloud parameters
		self.token_validity = 3600
		self.token_url = "https://oauth2.googleapis.com/token"
		self.scopes = "https://www.googleapis.com/auth/cloud-platform"
		self.api_endpoint = "https://texttospeech.googleapis.com/v1/text:synthesize"
		self.service_account = service_account.Credentials.from_service_account_info(json.loads(os.getenv("TTS_SERVICE_ACCOUNT")))


		self.publisher = self.create_publisher(Bool, 'conversation/reset', 10)            
		self.response_subscriber = self.create_subscription(String, 'conversation/response', self.talk, 10)
		self.parameter_subscriber = self.create_subscription(String, 'conversation/parameters', self.change_property, 10)
		self.manager_subscriber = self.create_subscription(Bool, 'manager/nodes_initialized', self.manager_talk, 10)
		self.srv = self.create_service(TalkString, 'talk', self.talk_srv)

		self.get_logger().info('Talker node initialized.')

	def change_property(self, msg):
		try:
			params = self.parse_params(msg)
			self.change_parameter(params)
			self.gender = self.get_parameter('gender').get_parameter_value().string_value.upper()

			if self.gender == 'FEMALE':
				self.accent = 'en-US-Neural2-C'

			if self.gender == 'MALE':
				self.accent = 'en-US-Neural2-D'

			self.frequency = round(1 / self.get_parameter('fps').get_parameter_value().integer_value, 2)

			# self.get_logger().info(f'M: {self.model}')
			self.get_logger().info(f'F: {self.frequency}')
			
		except:
			return
		
	def talk_srv(self, request, response):
		response.success = self.talk(request.text)
		self.get_logger().info(f'Incoming request received by Talker: {request.text}')


	def reset_conversation(self):
		msg = Bool()
		msg.data = True
		self.publisher.publish(msg)
		self.get_logger().info('Playback finished. Resetting conversation...')

	def talk(self, msg):
		""" Talks given a response in text. """
		if type(msg) is String:
			msg = msg.data
		if type(msg) != str:
			self.get_logger().info('The provided message cannot be parsed.')

		if self.inference == 'local':
			self.talk_local(msg, voice='af_heart')

		if self.inference == 'cloud':
			self.talk_cloud(msg)

		self.reset_conversation()

		return True

	def play_audio(self, audio: torch.Tensor, sample_rate):
		audio = audio.cpu().numpy()
		sd.play(audio, samplerate=sample_rate)
		sd.wait()

	def talk_local(self, msg: str, voice):
		pipeline = KPipeline(lang_code='a')
		generator = pipeline(
			msg,
			voice=voice,
		)
		
		graphemes, phonemes, audio = next(generator)
		self.play_audio(audio, 24000)

	def manager_talk(self, msg):
		self.talk('All nodes have been initialized, what can I do for you?')

	### CLOUD

	def write_json(self, text):
		""" Writes JSON payload for Text-to-Speech Inference """
		
		data = {
			"input": {
				"text": f"{text}"
			},
			"voice": {
				"languageCode": f"{self.language}",
				"name": f"{self.accent}",
				"ssmlGender": f"{self.gender}"
			},
			"audioConfig": {
				"audioEncoding": f"{self.encoding_format}"
			}
		}

		return json.dumps(data)

	def vocalize(self, data):
		""" Returns speech of a given API response. """

		now = int(time.time())
		payload = {
			"iss": self.service_account.service_account_email,
			"scope": self.scopes,
			"aud": self.token_url,
			"iat": now,
			"exp": now + self.token_validity
		}

		# Sign the JWT using the private key from the service account
		signed_jwt = google_jwt.encode(crypt.RSASigner.from_service_account_info(json.loads(os.getenv('TTS_SERVICE_ACCOUNT'))), payload)

		# Request access token
		response = requests.post(self.token_url, data={
			"grant_type": "urn:ietf:params:oauth:grant-type:jwt-bearer",
			"assertion": signed_jwt
		})

		# Check response
		if response.status_code == 200:
			access_token = response.json()["access_token"]

			# Use the access token to call the Text-to-Speech API
			api_headers = {
				"Authorization": f"Bearer {access_token}",
				"Content-Type": "application/json; charset=utf-8"
			}

			# Make the POST request
			api_response = requests.post(self.api_endpoint, headers=api_headers, data=data)

			api_response_text = api_response.text

			# Print a message indicating completion
			if api_response.status_code == 200:
				self.get_logger().info(f"Response saved.")
			else:
				self.get_logger().error(f"API Error: {api_response.status_code} {api_response.text}")
		else:
			self.get_logger().error(f"Error: {response.status_code} {response.text}")

		

		return api_response_text

	def talk_cloud(self, msg):
		""" Talks given a response in text. """

		# Request a text-to-speech response
		if isinstance(msg, str):
			response = self.write_json(msg)
		else:
			response = self.write_json(msg.data)
		speech = self.vocalize(response)
		# self.save_audio(speech)

		# Parse the API response and decode base64 audio content
		response_json = json.loads(speech)
		audio_data = base64.b64decode(response_json['audioContent'])

		# Use io.BytesIO to handle audio data in memory
		audio_stream = io.BytesIO(audio_data)
		with wave.open(audio_stream, 'rb') as wav_file:
			# Set up the PyAudio stream
			audio = pyaudio.PyAudio()
			stream = audio.open(
				format=audio.get_format_from_width(wav_file.getsampwidth()),
				channels=wav_file.getnchannels(),
				rate=wav_file.getframerate(),
				output=True
			)

			# Read and play audio data
			data = wav_file.readframes(1024)
			while data:
				stream.write(data)
				data = wav_file.readframes(1024)

			# Stop and close the stream
			stream.stop_stream()
			stream.close()
			audio.terminate()

def main(args=None):
	load_dotenv()

	try:
		rclpy.init(args=args)
		lone_talker = Talker()
		rclpy.spin(lone_talker)
	except KeyboardInterrupt:
		pass

	lone_talker.destroy_node()

if __name__ == '__main__':
	main()