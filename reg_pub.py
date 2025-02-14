#!/usr/bin/env python
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, Dataset
from torchvision import models, transforms
# from PIL import Image
from sensor_msgs.msg import Image
# import matplotlib.pyplot as plt
import numpy as np
import rospy
import torchvision.transforms as transforms
from std_msgs.msg import Int32, Float64  
from cv_bridge import CvBridge, CvBridgeError
import torchvision.models as models

'''
Work with trained 'ResNet18-LSTM' model, timestep = 2

'''
'''
This node will be able to process next step action prediction based on current state (realtime image), with a trained 'ResNet18-LSTM' model.
Then the predicted outputs (delta x y z) will be published through "reg_results".
'''


class ImageRegressionLSTM(nn.Module):
    def __init__(self, hidden_size, num_layers, num_regression_output):
        super(ImageRegressionLSTM, self).__init__()

        # Load pretrained ResNet-18
        self.resnet18 = models.resnet18(pretrained=True)
        self.resnet18.fc = nn.Identity()  # Remove fc layer

        # Extract features using ResNet-18
        num_features = 512 

        self.lstm_layers = nn.LSTM(
            input_size=num_features, # 512 by default, output of resnet18
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True
        )

        self.fc = nn.Linear(hidden_size, num_regression_output)  # Output: (x, y, z)

    def forward(self, x):
        batch_size, timesteps, C, H, W = x.size()

        # Reshape input 
        x = x.view(batch_size * timesteps, C, H, W) 
        features = self.resnet18(x)  # Extract features 512

        features = features.view(batch_size, timesteps, -1)  
        lstm_out, _ = self.lstm_layers(features)
        lstm_last_output = lstm_out[:, -1]  

        output = self.fc(lstm_last_output)
        return output


class InferenceCallback:
    def __init__(self, model, transform, delta_x_pub, delta_y_pub, delta_z_pub):
        self.model = model
        self.transform = transform
        self.delta_x_pub = delta_x_pub
        self.delta_y_pub = delta_y_pub
        self.delta_z_pub = delta_z_pub

    def process_image(self, img):
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        input_image = self.transform(cv_image).unsqueeze(0).unsqueeze(1)  # Add a single timestep dimension
        with torch.no_grad():
            output = self.model(input_image)

        output_x, output_y, output_z = output[0]
        #rospy.loginfo(f"Predicted delta x: {output_x.item()}, delta y: {output_y.item()}, delta z: {output_z.item()}")
        
        # Denormalization
        min_x, max_x = -0.0141, 0.0245  # Original range for output_x
        min_y, max_y = -0.0129, 3.0802e-05  # Original range for output_y
        min_z, max_z = -2.1892e-05, 0.0118  # Original range for output_z

        output_x = -10*((output_x + 1) * (max_x - min_x) / 2 + min_x)+0.1
        output_y = (output_y + 1) * (max_y - min_y) / 2 + min_y+0.0079
        output_z = (output_z + 1) * (max_z - min_z) / 2 + min_z+0.03

        # Publish regression results
        self.delta_x_pub.publish(output_x.item())
        self.delta_y_pub.publish(output_y.item())
        self.delta_z_pub.publish(output_z.item())

        print("Published regression results", output_x.item(), output_y.item(), output_z.item())

    def callback(self, data):
        self.process_image(data)

if __name__ == "__main__":
    rospy.init_node("inference_node")

    # Init model
    hidden_size = 16
    num_layers = 2
    num_regression_output = 3
    model = ImageRegressionLSTM(hidden_size, num_layers, num_regression_output)
    reg_saved_model_path = "/root/ros_ws/src/ur10e_robotiq/amiga_moveit_wrapper/scripts/reslstm_reg.pth"
    model.load_state_dict(torch.load(reg_saved_model_path, map_location=torch.device('cpu')))
    model.eval()

    print(model)

    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.ToTensor(),
    ])

    image_topic = "/zed2_node/rgb/image_rect_color"
    args = (model, transform)  

    delta_x_pub = rospy.Publisher('delta_x', Float64, queue_size=10)
    delta_y_pub = rospy.Publisher('delta_y', Float64, queue_size=10)
    delta_z_pub = rospy.Publisher('delta_z', Float64, queue_size=10)

    inference_callback = InferenceCallback(model, transform, delta_x_pub, delta_y_pub, delta_z_pub)
    rospy.Subscriber(image_topic, Image, inference_callback.callback)

    rospy.spin()
