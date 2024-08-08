import rosbag
import rospy
import sys

def cut_bag_from_end(input_bag_path, output_bag_path, duration_from_end):
    # Open the original bag file
    with rosbag.Bag(input_bag_path, 'r') as input_bag:
        # Get the start and end times of the original bag
        start_time = input_bag.get_start_time()
        end_time = input_bag.get_end_time()
        print(f"Original bag start time: {start_time}")
        print(f"Original bag end time: {end_time}")

        # Calculate the start time for the new bag
        new_start_time = end_time - duration_from_end
        if new_start_time < start_time:
            new_start_time = start_time
            print("Specified duration exceeds the bag duration. Cutting from the start of the bag.")

        # Open the new bag file for writing
        with rosbag.Bag(output_bag_path, 'w') as output_bag:
            for topic, msg, t in input_bag.read_messages(start_time=rospy.Time(new_start_time)):
                output_bag.write(topic, msg, t)

        print(f"New bag file created: {output_bag_path}")

if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python cut_bag.py <input_bag_path> <output_bag_path> <duration_from_end>")
        sys.exit(1)

    input_bag_path = sys.argv[1]
    output_bag_path = sys.argv[2]
    duration_from_end = float(sys.argv[3])

    cut_bag_from_end(input_bag_path, output_bag_path, duration_from_end)

