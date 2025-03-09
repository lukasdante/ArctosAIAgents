import pyaudio
import wave
import numpy
import io
import base64
from ctypes import *

from utils.nodes import BaseNode
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int64
from rcl_interfaces.msg import ParameterDescriptor

class Recorder(BaseNode):
    def __init__(self):
        super().__init__('recorder')
        
        try:
            self.declare_parameter('chunk_size', 1024,
                                   ParameterDescriptor(description='Chunk size of the audio recording.'))
            self.declare_parameter('channels', 1,
                                   ParameterDescriptor(description='Number of channels of the audio recording.'))
            self.declare_parameter('sample_rate', 16000,
                                   ParameterDescriptor(description='Sample rate of the audio recording.'))
            self.declare_parameter('threshold', 1000,
                                   ParameterDescriptor(description='Silence volume threshold of audio recording for automatic termination.'))
            # MAKE SURE THE SILENCE_LIMIT HAS DECIMAL
            self.declare_parameter('silence_limit', 2.5,
                                   ParameterDescriptor(description='Silence length until audio recording terminates.'))

            self.chunk_size = self.get_parameter('chunk_size').get_parameter_value().integer_value
            self.channels = self.get_parameter('channels').get_parameter_value().integer_value
            self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
            self.threshold = self.get_parameter('threshold').get_parameter_value().integer_value
            self.silence_limit = self.get_parameter('silence_limit').get_parameter_value().double_value

            self.publisher = self.create_publisher(String, 'conversation/recording', 10)
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


    def record(self, msg):
        """ Record audio until silence is detected. """
        if not msg.data:
            self.get_logger().info("Recording stopped. Conversation not ready for new instance.")
            return

        audio = pyaudio.PyAudio()

        # Open audio stream for recording
        try:
            stream = audio.open(format=pyaudio.paInt16,
                            channels=self.channels,
                            rate=self.sample_rate,
                            input=True,
                            frames_per_buffer=self.chunk_size)
            self.get_logger().info("Recording started. Speak into the microphone...")
        except Exception as e:
            self.get_logger().error(f"Failed to open audio stream: {e}.")
            return

        # Read and store the audio stream until silence is detected
        frames = []
        silent_chunks = 0
        last_log_time = self.get_clock().now().nanoseconds
        initial_time = self.get_clock().now().nanoseconds
        while True:
            try:
                data = stream.read(self.chunk_size)
            except Exception as e:
                self.get_logger().error(f"Error reading audio data: {e}")
                break

            frames.append(data)
            
            # Calculate the volume
            volume = self.get_volume(data)

            # For every 200ms display the volume
            volume_msg = String()
            volume_msg.data = f'Current volume: {int(volume)} -> {self.threshold}'
            self.volume_pub.publish(volume_msg)
            # if (self.get_clock().now().nanoseconds - last_log_time) >= (0.2 * 1e9):
            #     self.get_logger().debug(f"Current volume: {volume}")
            #     last_log_time = self.get_clock().now().nanoseconds
            
            # Record silent chunks and stop recording once limit is reached
            if volume < self.threshold:
                silent_chunks += 1
            else:
                silent_chunks = 0
            
            if silent_chunks >= int(self.silence_limit * self.sample_rate / self.chunk_size + 0.05):
                # If audio is not entirely silent publish it
                if (self.get_clock().now().nanoseconds - initial_time) > ((self.silence_limit + 0.1) * 1e9):
                    self.get_logger().info("Silence detected. Stopping recording...")
                    break
                    
                
        # Stop and close the stream
        stream.stop_stream()
        stream.close()
        audio.terminate()

        # Create a BytesIO buffer to store WAV data in memory
        wav_buffer = io.BytesIO()

        # Write the WAV data into the buffer
        with wave.open(wav_buffer, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(audio.get_sample_size(pyaudio.paInt16))
            wf.setframerate(self.sample_rate)
            wf.writeframes(b''.join(frames))

        # Get the WAV data from the buffer
        wav_buffer.seek(0)
        audio_content = wav_buffer.read()
    
        # Prepare the message
        msg = String()
        msg.data = base64.b64encode(audio_content).decode('utf-8')
        
        # Publish message to topic `conversation/request_audio`
        self.publisher.publish(msg)

    def get_volume(self, data):
        """ Obtains the mean absolute value of the current audio. """

        if not data:
            return 0
        audio_data = numpy.frombuffer(data, dtype=numpy.int16)
        return numpy.abs(audio_data).mean()

def main(args=None):
    # Handle ASLA errors for cleaner output.
    ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
    def py_error_handler(filename, line, function, err, fmt):
        pass
    c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
    asound = cdll.LoadLibrary('libasound.so')
    asound.snd_lib_error_set_handler(c_error_handler)

    try:
        rclpy.init(args=args)

        lone_recorder = Recorder()

        rclpy.spin(lone_recorder)
    
    except KeyboardInterrupt:
        pass

    lone_recorder.destroy_node()

if __name__ == '__main__':
    main()