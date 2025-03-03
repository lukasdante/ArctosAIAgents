import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String, Bool
from utils.nodes import BaseNode
from interfaces.srv import TalkString

from kokoro import KPipeline
import soundfile as sf
import sounddevice as sd
import torch

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
		# self.token_validity = self.get_parameter('token_life').get_parameter_value().integer_value
		self.inference = self.get_parameter('inference').get_parameter_value().string_value
		self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value

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
			self.talk_cloud()

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

	def talk_cloud(self, msg: str):
		self.play_audio()

	def manager_talk(self, msg):
		self.talk('All nodes have been initialized what can I do for you?')

def main(args=None):

	try:
		rclpy.init(args=args)
		lone_talker = Talker()
		rclpy.spin(lone_talker)
	except KeyboardInterrupt:
		lone_talker.destroy_node()
		pass

if __name__ == '__main__':
	main()