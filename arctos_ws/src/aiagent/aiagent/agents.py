import os
import uuid
import json
from pathlib import Path
from dotenv import load_dotenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterDescriptor
from interfaces.msg import Response

from utils.nodes import BaseNode
from google.cloud.dialogflowcx_v3beta1.services.agents import AgentsClient
from google.cloud.dialogflowcx_v3beta1.services.sessions import SessionsClient
from google.cloud.dialogflowcx_v3beta1.types import session
from google.api_core.client_options import ClientOptions
from google.protobuf.json_format import MessageToDict
from google.oauth2 import service_account

class Agent(BaseNode):
	def __init__(self):
		super().__init__('parser')

		# try:
		self.declare_parameter('language', 'en-US', ParameterDescriptor(description='Language of the transcription input.'))
		self.declare_parameter('mode', 'playbook', ParameterDescriptor(description='Mode of the intent detection.'))
		self.declare_parameter('inference', 'cloud', ParameterDescriptor(description="The inference method, can be 'local' or 'cloud'."))

		self.language = self.get_parameter('language').get_parameter_value().string_value
		self.mode = self.get_parameter('mode').get_parameter_value().string_value
		

		self.project_id = os.getenv("DFCX_PROJECT_ID")
		self.location_id = os.getenv("DFCX_LOCATION_ID")
		self.nlp_agent_id = os.getenv("DFCX_AGENT_ID")
		self.llm_agent_id = os.getenv("DFCX_LLM_AGENT_ID")
		self.service_account = os.getenv("DFCX_SERVICE_ACCOUNT")

		self.inference = self.get_parameter('inference').get_parameter_value().string_value

		self.service_account = json.loads(self.service_account)
		self.service_account = service_account.Credentials.from_service_account_info(self.service_account)

		if self.mode == 'playbook':
			self.agent = f'projects/{self.project_id}/locations/{self.location_id}/agents/{self.llm_agent_id}'
		else:
			self.agent = f'projects/{self.project_id}/locations/{self.location_id}/agents/{self.nlp_agent_id}'
		
		self.session_id = None

		self.response_publisher = self.create_publisher(String, 'conversation/response', 10)
		self.params_publisher = self.create_publisher(String, 'conversation/parameters', 10)
		self.subscriber = self.create_subscription(String, 'conversation/transcript', self.detect_intent, 10)
		# TODO: removed parameter subscriber, do in-house property change. it has callback to change_property
		# TODO: do actor publisher
		# self.actor_params_publisher = self.create_publisher(String, 'actor/conversation/parameters', 10)

		self.get_logger().info("Agent initialized.")
		# except Exception as e:
		#     self.get_logger().error(f"Unable to initialize parser: {e}")

	def change_property(self, msg):
		try:
			params = self.parse_params(msg)
			self.change_parameter(params)

			if self.get_parameter('mode').get_parameter_value().string_value == "Dialogflow CX":
				self.mode = "flows"

			if self.get_parameter('mode').get_parameter_value().string_value == "Conversational Agents":
				self.mode = "playbook"

		except:
			return
		
	
	def detect_intent(self, msg: String):
		""" Transcribes an audio from base64-encoded string to string. """

		msg = msg.data

		# Runs local inference
		if self.inference == 'local':
			self.detect_intent_local(msg)

		# Runs cloud inference
		if self.inference == 'cloud':
			self.detect_intent_cloud(msg)

	
	
	def detect_intent_cloud(self, msg):
		""" Detect intent, extract parameters, and output response. """

		if not self.session_id:
			self.session_id = uuid.uuid4()
		
		session_path = f'{self.agent}/sessions/{self.session_id}'
		api_endpoint = f'{AgentsClient.parse_agent_path(self.agent)["location"]}-dialogflow.googleapis.com:443'
		client_options = ClientOptions(api_endpoint=api_endpoint)
		session_client = SessionsClient(client_options=client_options)

		# Configure the request
		if isinstance(msg, str):
			text_input = session.TextInput(text=msg)
		else:
			text_input = session.TextInput(text=msg.data)
		query_input = session.QueryInput(text=text_input, language_code=self.language)
		
		# Send the request
		request = session.DetectIntentRequest(
			session=session_path, query_input=query_input
		)

		# Obtain the response
		response = session_client.detect_intent(request=request)

		# 
		response_messages = [
			" ".join(msg.text.text) for msg in response.query_result.response_messages
		]

		# Prepare response message data
		response_text = ' '.join(response_messages)

		# If playbook mode return the parsed responses and parameters
		if self.mode == 'playbook':
			# Extract the response and the parameters
			response, params = self.extract_params_response(response_text)

			# Publish the response
			response_msg = String()
			response_msg.data = response
			self.response_publisher.publish(response_msg)
			self.get_logger().info(f"Response text: {response_text}")
			
			

			# Publish the parameters
			params_msg = String()
			params_msg.data = json.dumps(params)
			self.params_publisher.publish(params_msg)
			# TODO: configure actor publisher
			# self.actor_params_publisher.publish(params_msg)
			self.get_logger().info(f"Parameters: {params}")

			return {'response': response, 'parameters': params}

		# Convert the parameters to a dictionary and prepare response
		try:
			# Prepare the response message data
			response_msg = String()
			response_msg.data = response_text

			# Publish response message
			self.response_publisher.publish(response_msg)
			self.get_logger().info(f"Response text: {response_text}")

			# Extract parameters
			parameters = MessageToDict(response._pb)
			self.get_logger().info(f"Parameters: {parameters['queryResult']['parameters']}")

			# Prepare parameters message data
			params_msg = String()
			params_msg.data = json.dumps(parameters)

			# Publish parameters message
			self.params_publisher.publish(params_msg)
			self.actor_params_publisher.publish(params_msg)
			self.get_logger().info(f"Parameters: {parameters['queryResult']['parameters']}")

			return {'response': response_text, 'parameters': parameters['queryResult']['parameters']}
		
		except Exception as e:
			self.get_logger().warn(f'Failed to detect intent {e}')

	def detect_intent_local(self):
		pass

	def extract_params_response(self, text: str):
		parameters_dict = {}

		if 'Parameters' in text:
			response, parameters = [value.strip() for value in text.split('Parameters: ')]
			parameters = parameters.strip().split(';')
		
			for param in parameters:
				key, value = param.split('-')
				parameters_dict[f'{key.strip()}'] = value.strip()
		else:
			response = text.strip()

		return response, parameters_dict

		

def main(args=None):
	load_dotenv()

	try:
		rclpy.init(args=args)

		lone_parser = Agent()

		rclpy.spin(lone_parser)

	except KeyboardInterrupt:
		pass

	lone_parser.destroy_node()


if __name__ == '__main__':
	main()