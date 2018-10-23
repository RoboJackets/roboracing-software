import rosbag
import sys
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

"""
	Converts images stored in bag files to a video. Defaults to a 30 fps mp4 but
	that can be configured via command line arguments. The video is saved in the
	working directory with the same name as the bag file.
	
	Uses the RosBag and OpenCV python libraries
	
	Ex:
		python bag_converter.py some_bag.bag
			|-> some_bag.mp4
		python bag_converter.py some_bag.bag encoding=mjpg extension=.avi fps=30
			|-> some_bag.avi
"""

def do_conversion():
	if len(sys.argv) < 2:
		print("Please specify which bag file to convert")
		return

	target = sys.argv[1]
	if ".bag" not in target:
		print("Only .bag files are supported")
		return
	
	# default values
	encoding = "mp4v"
	extension = ".mp4"
	fps = 30
	
	for arg in sys.argv:
		if "encoding=" in arg:
			encoding = arg.split("=")[1]
		elif "extension=" in arg:
			extension = arg.split("=")[1]
		elif "fps=" in arg:
			fps = int(arg.split("=")[1])
		elif "=" in arg:
			print("Unknown argument %s" % arg)

	try:
		open(target)
	except:
		print("Specified file not found")
		return

	print("Trying to convert %s with encoding %s, extension %s, and %s fps"
		% (target, encoding, extension, fps))

	cv_bridge = CvBridge()
	video_name = target.replace(".bag", extension)
	video = None
	
	print("Converting bag file to video...")

	bag = rosbag.Bag(target)
	for topic, msg, t in bag.read_messages(topics="/camera/image_color_rect_flipped"):
		try:
			if video is None:
				frame_size = (msg.width, msg.height)
				video = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*encoding), fps, frame_size)
			image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
			video.write(image)
		except e:
			print("Failed to read image from bag: " + str(e))
			return

	cv2.destroyAllWindows()
	video.release()
	print("Conversion complete!")
	
	

do_conversion()
