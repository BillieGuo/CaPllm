
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
		self.voice_input_pub = self.create_publisher(
			String,
			'llm_to_voice',
			10)
		self.vision_response_str_sub = self.create_subscription(
            String,
            'yolo_detections_str',
            self.vision_response_callback,
            10
        )
		self.voice_input_sub
		self.voice_input = ""
		self.vision_info = []
		self.vision_history = []
		self.vision_update = False
		self.detection_fail = False

	# Receive response from Basemotion
	def query_callback(self, msg):
		# self.get_logger().info('BaseMotion: "%s"' % msg)
		return
  	# original version
	def vision_response_callback(self, msg):
		self.get_logger().info(f'vision response: {msg.data}')
		if msg.data:
			if msg.data == "Target not found":
				self.detection_fail = True
				self.vision_update = True

			elif msg.data == "Object found":
				self.detection_fail = False
				self.vision_update = True
			else:
				self.vision_info = list(eval(msg.data))
				for obj in self.vision_info:
					if obj not in self.vision_history:
						self.vision_history.append(obj)
				self.vision_update = True

	# def vision_response_callback(self, msg):
	# 	self.get_logger().info(f'vision response: {msg.data}')
	# 	if msg.data:
	# 		if msg.data == "Target not found":
	# 			self.detection_fail = True
	# 		self.vision_info = list(eval(msg.data))

	# 		for obj in self.vision_info:
	# 			if obj not in self.vision_history:
	# 				self.vision_history.append(obj)
	# 		self.vision_update = True

	# Publish command to Basemotion
	def publish_cmd(self, cmd):
		msg = String()
		msg.data = cmd
		self.query_pub.publish(msg)

	# Receive voice input
	def voice_input_callback(self, msg):
		self.voice_input = ""
		if msg:
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
	# base_motion = BASEMOTION()
	query = QUERY()
	# history stored in format of [input_text, result]
	# max length of query history is 10
	query_history = {0: ["", ""]}
	query_history_max_len = 3
	query_history_idx = 0
	action_history = {0: ""}
	action_history_max_len = 20
	action_history_idx = 0

	# Main loop
	while True:
		try:
			# input_text = input("\n\033[1;33m>> Prompt: ")
			# print("\033[0m")
			# rclpy.spin_once(query, timeout_sec=0.1)
			input_text = None
			rclpy.spin_once(query)
			input_text = query.voice_input
			# input_text = "turn left for 180 degree and Go forward." # test only
			
			if not input_text:
				continue
			if input_text == 'exit':
				break
			if input_text in stopping_vocab_list:
				query.publish_cmd("stop")
				continue

			success = False
			while not success:

				# history stored in format of [input_text, result]
				if query.vision_info or query.vision_history:
					input_text = f'Throughout the history, the total object you see are: {query.vision_history}; The follow object(s) is in your current vision view: {query.vision_info}; the user said: {input_text}'
				result, success = preview(input_text)  
				print(result, success)
				print("*"*80)
				try:
					intermediate = list(eval(result))
					print(intermediate)
					if intermediate[0] == "CHAT":
						print("\033[1;31m>> Chat: ", intermediate[2], "\033[0m")
						print("*"*80)
						tx = String()
						tx.data = intermediate[2]
						query.voice_input_pub.publish(tx)
					elif intermediate[0] == "MIXED":
						for cmd in range(2, len(intermediate)):
							if intermediate[cmd][0] == "CHAT":
								print("\033[1;31m>> Chat: ", intermediate[cmd][2], "\033[0m")
								print("*"*80)
								tx = String()
								tx.data = intermediate[cmd][2]
								query.voice_input_pub.publish(tx)
				except Exception as e:
					query.get_logger().info(f'Error: {e}')
					continue
 
				# input_text = [intermediate[0], intermediate[2:]]
				model_input = f'Query: {intermediate}'
				# model_input = f'Query: {input_text}'
				print(model_input)
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
				time.sleep(1)

				try: 
					results = list(eval(result))
				except Exception as e:
					query.get_logger().info(f'Error: {e}')
					print(result)
					continue
				if results[0][0] == 20:
					query.vision_update = False
					while not query.vision_update:
						rclpy.spin_once(query)
						if query.detection_fail:
							print("\033[1;31m>> Chat: Sorry, I can not find it.\033[0m")
							query.detection_fail = False
							break
		except KeyboardInterrupt:
			print("\033[0m")
			print("KeyboardInterrupt")
			break
		

	rclpy.shutdown()
 
if __name__ == '__main__':
    main()
