
from capllm.LMP import LMP
from capllm.utils import get_config
from capllm.BaseMotion import BASEMOTION
import numpy, subprocess, time 

import os
import openai
from openai import AzureOpenAI
from dotenv import load_dotenv
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

stopping_vocab_list = ['stop', 'Stop', 'STOP', 'Stop.', 'break', 'Break', 'BREAK', 'Break.', 'exit', 'Exit', 'EXIT', 'Exit.', 'Quit', 'quit', 'QUIT', 'Quit.']

class QUERY(Node):
	def __init__(self):
		super().__init__('Query')
		self.query_pub = self.create_publisher(
			String,
			'query',
			10)
		self.response_sub = self.create_subscription(
			String, 
			'response', 
			self.query_callback, 
			10)
		self.response_sub
		self.voice_input_sub = self.create_subscription(
			String,
			'voice_input',
			self.voice_input_callback,
			10)
		self.voice_input_sub

	# Receive response from LLM
	def query_callback(self, msg):
		self.get_logger().info('BaseMotion: "%s"' % msg)	
		
	# Publish command to LLM
	def publish_cmd(self, cmd):
		msg = String()
		msg.data = cmd
		self.query_pub.publish(msg)

	# Receive voice input
	def voice_input_callback(self, msg):
		self.get_logger().info('I heard: "%s"' % msg)	
		self.voice_input = msg.data

def model_init():
	cfg = get_config('capllm/configs/config.yaml')['lmps']
	fixed_vars = {'numpy':numpy, 'subprocess':subprocess, 'time': time} # for third libraries that LLM can access
	variable_vars = {} # for first party libraries (can be other LLM) that a LLM can access
	# allow LMPs to access other LMPs
	# & low-level LLM setup
	lmp_names = [name for name in cfg.keys() if not name in ['coder','previewer']] # cfg=lmps_config
	low_level_lmps = {
		k: LMP(k, cfg[k], fixed_vars, variable_vars) #, debug, env_name)
		for k in lmp_names
	}
	variable_vars.update(low_level_lmps)

	# high-level LLM setup
	coder = LMP("coder", cfg['coder'], fixed_vars, variable_vars)
	previewer = LMP("previewer", cfg['previewer'])
	# previewer = None # 
 
	return previewer, coder

def main():
	# declare global variables

	# init
	rclpy.init()
	preview, model = model_init() 
	base_motion = BASEMOTION()
	query = QUERY()
	# history stored in format of [input_text, result]
	# max length of query history is 10
	query_history = {0: ["", ""]}
	query_history_max_len = 3
	query_history_idx = 0
	action_history = {0: ""}
	action_history_max_len = 20
	action_history_idx = 0

	while True:
	# if True: # test only
		input_text = input("\n>>Prompt: ")
		# rclpy.spin_once(query)
		# input_text = query.voice_input
		# input_text = "turn left for 180 degree and Go forward." # test only
		if not input_text:
			continue
		if input_text == 'exit':
			break
		if input_text in stopping_vocab_list:
			query.publish_cmd("stop")
			continue

		# Main loop
		success = False
		while not success:

			# history stored in format of [input_text, result]
			print("*"*80)
			result, success = preview(input_text)  
			print(result, success)
			print("*"*80)
			# model_input = f'Query: {input_text}'
			model_input = f'Query: {result}'
			# print("*"*80)
			# print(model_input)
			print("*"*80)
			result, success = model(model_input)
			print(result, success)
			print("*"*80)
			# input()
			# continue
			if success:
				# query_history_idx = (query_history_idx + 1) % query_history_max_len
				# query_history[query_history_idx] = [input_text, result]
				query.publish_cmd(result)
				query.voice_input = None

	rclpy.shutdown()
 
if __name__ == '__main__':
    main()
