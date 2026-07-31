import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import re

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.publisher = self.create_publisher(String, '/voice_command', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.shapes = ["circle", "square", "rectangle", "triangle"]
        self.last_command = None

        self.get_logger().info("Voice Command Node Started")
        self.create_timer(3.0, self.listen)

    def listen(self):
        with self.microphone as source:
            self.get_logger().info("Listening...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                audio = self.recognizer.listen(source, timeout=3)
                text = self.recognizer.recognize_google(audio).lower()
                self.get_logger().info(f"You said: {text}")

                # Match "circle 1", "square 2", "triangle 3" etc.
                for shape in self.shapes:
                    # Look for shape followed by a number (spoken or digit)
                    pattern = rf'{shape}\s+(\w+)'
                    match = re.search(pattern, text)
                    if match:
                        raw_num = match.group(1)
                        number = self._word_to_int(raw_num)
                        if number is not None:
                            command = f'{shape}_{number}'
                            if command != self.last_command:
                                self.publish_command(command)
                                self.last_command = command
                            return

                self.get_logger().info("No valid shape+number detected")

            except sr.WaitTimeoutError:
                self.get_logger().warn("Listening timeout")
            except sr.UnknownValueError:
                self.get_logger().warn("Could not understand audio")
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def _word_to_int(self, word):
        """Convert spoken number words or digit strings to int."""
        word_map = {
            'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
            'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10
        }
        if word in word_map:
            return word_map[word]
        try:
            return int(word)
        except ValueError:
            return None

    def publish_command(self, command):
        msg = String()
        msg.data = command   # e.g. "circle_1"
        self.publisher.publish(msg)
        self.get_logger().info(f"Published command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()