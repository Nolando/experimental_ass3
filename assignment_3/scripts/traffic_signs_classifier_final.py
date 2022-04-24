#! /usr/bin/env python
#
# Group: Machine Unlearning
# Description: 

import pickle
import matplotlib.pyplot as plt
import cv2
import torch
import torch.nn as nn
import time
import numpy as np
import math

from torch import nn, optim
from torch.utils.data import Dataset, DataLoader
import torchvision
from torchvision import transforms

training_file = "traindata.pkl"
validation_file = "validdata.pkl"
testing_file = "testdata.pkl"

with open(training_file, mode='rb') as f:
    train = pickle.load(f)
with open(validation_file, mode='rb') as f:
    valid = pickle.load(f)
with open(testing_file, mode='rb') as f:
    test = pickle.load(f)

x_train, y_train = train['features'], train['labels']
x_valid, y_valid = valid['features'], valid['labels']
x_test, y_test = test['features'], test['labels']

# Custom Dataloader Class
class TrafficSignsDataset(Dataset):

    def __init__(self, features, labels, transform=None):
        self.features = features
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        if torch.is_tensor(idx):
            idx = idx.tolist()

        x = self.features[idx]
        y = self.labels[idx]

        if self.transform:
            x = self.transform(x)
            y = y

        return x, y






# Define custom transforms to apply to the images
# ------------------------------------------------
# Here we are applying a normalization to bring all pixel values between 0 and 1
# and converting the data to a pytorch tensor
custom_transform = transforms.Compose([transforms.ToTensor(),
                                       transforms.Normalize((0.5, ), (0.5, ))])

# Define the dataloaders
# -----------------------
# Dataloaders help in managing large amounts of data efficiently. You should define a set batch size here as part of your training regime.
train_dataset = TrafficSignsDataset(x_train, y_train,
                                    custom_transform)
train_loader = DataLoader(train_dataset, shuffle=True,batch_size=??)

validation_dataset = TrafficSignsDataset(x_valid, y_valid,
                                         custom_transform)
validation_loader = DataLoader(validation_dataset, shuffle=True)

test_dataset = TrafficSignsDataset(x_test, y_test,
                                   custom_transform)
test_loader = DataLoader(test_dataset, shuffle=True,batch_size=??)


#####################################################################################################
# WRITE YOUR CODE FROM HERE. REFER TO THE LECTURE SLIDES FOR HELP.
# YOU CAN ALSO REFER TO RESOURCES ONLINE FOR HELP.
# FOR YOUR CONVENIENCE THE TASK HAS BEEN SPLIT INTO STEPS.

#####################################################################################################
# Step 1: Knowing your dataset
#---------------------------------
# Write code here to visualize the images, print out dimensions of the training images,
# Printing out ground truth labels for these images, batch size, etc.
# In the above dataloaders, we do not set a batch size. feel free to experiment.
#####################################################################################################

#####################################################################################################
# Step 2: Building your Neural Network
#---------------------------------------
# Write code here to build your neural network. This has been scaffolded for you.
# We also define a BasicBlock class which may assist in building the model, which you will need to define some values for.
#
# Some modules you can use are
# torch.nn.Sequential()
# torch.nn.Conv2d()
# torch.nn.Linear()
# torch.nn.ReLU()
#
# model = 
#
#####################################################################################################

class BasicBlock(nn.Module):
    
    expansion = 1
    
    def __init__(self, in_channels, out_channels, stride = 1, downsample = False):
        super().__init__()
                
        self.conv1 = nn.Conv2d(in_channels, out_channels, kernel_size = ??, 
                               stride = stride, padding = 1, bias = False)
        self.bn1 = nn.BatchNorm2d(out_channels)
        
        self.conv2 = nn.Conv2d(out_channels, out_channels, kernel_size = ??, 
                               stride = 1, padding = 1, bias = False)
        self.bn2 = nn.BatchNorm2d(out_channels)
        
        self.relu = nn.ReLU(inplace = True)
        
        if downsample:
            conv = nn.Conv2d(in_channels, out_channels, kernel_size = 1, 
                             stride = stride, bias = False)
            bn = nn.BatchNorm2d(out_channels)
            downsample = nn.Sequential(conv, bn)
        else:
            downsample = None
        
        self.downsample = downsample
        
    def forward(self, x):
        
        i = x
        
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        
        x = self.conv2(x)
        x = self.bn2(x)
        
        if self.downsample is not None:
            i = self.downsample(i)
                        
        x += i
        x = self.relu(x)
        
        return x

class MTRX5700_NNet(nn.Module):
    '''
    This network is a modified architecture, adding in a fully connected classification head.
    '''
    def __init__(self, num_classes):
        super().__init__()
        self.in_channels = 64
        self.conv1 = nn.Conv2d(??, self.in_channels, kernel_size = ??, stride = ??, padding = 3, bias = False)
        self.bn1 = nn.BatchNorm2d(self.in_channels)
        self.relu = nn.ReLU(inplace = True)
        self.maxpool = nn.MaxPool2d(kernel_size = ??, stride = 2, padding = 1)
        
        self.layer1 = self.resnet_layer(BasicBlock, 2, 64)
        #TODO
        self.layer2 = ...
        self.layer3 = ...
        self.layer4 = ...

        
        self.classifier = self.classification_head(num_classes)
        
    def resnet_layer(self, block, n_blocks, channels, stride = 1):
    
        layers = []
        
        if self.in_channels != block.expansion * channels:
            downsample = True
        else:
            downsample = False
        
        layers.append(block(self.in_channels, channels, stride, downsample))
        
        for i in range(1, n_blocks):
            layers.append(block(block.expansion * channels, channels))

        self.in_channels = block.expansion * channels
            
        return nn.Sequential(*layers)

    def classification_head(self,num_classes):

        # In this section, we implement a classification head - using the ResNet backbone predominently for feature extraction.
        # We apply upsampling to the dense latent space given by the backbone, prior to using a Dropout layer to maximise robustness of the model
        # with the Bilinear interpolation of features.
        # We then perform adaptive average pooling across the 2D, prior to flattening.
        # The netowork then goes through 2 linear fully connected layers to perform classification.
        layers = []

	# TODO
        layers += ...

        

        return nn.Sequential(*layers)


        
    def forward(self, x):
        
        # TODO

        
        return x









#####################################################################################################
# Step 3: Defining loss function and optimizer
#-----------------------------------------------
# Define your loss function and optimizer here - refer to lecture slides for help.
# Explore different loss functions e.g. torch.nn.CrossEntropyLoss(), torch.nn.MSELoss(), torch.nn.NLLLoss()
# Explore different optimizers e.g. torch.optim.SGD(), torch.optim.Adam()
# Explore different values of learning rates and momentum.
#####################################################################################################
# loss_function =
# optimizer =


#####################################################################################################
# Step 4: Main training loop
#-----------------------------------------------
# Refer to lecture slides for help.
# Loop over your entire training data in 'epochs'
# For each epoch, get one minibatch from the dataset, perform forward pass, compute loss,
# backpropagation, adjust weight through optimization step
#
#####################################################################################################
# output = net(.....)
# loss = loss_function(.....)
# backprop
# optimizer step
#
# Print out the training loss and check that it is decreasing as the training progresses.
# Check validation loss after every few epochs and see that it is also decreasing.
#
# Save your model when done. Useful for question 5 part b.
# torch.save(net.state_dict(), "traffic_signs_classifier.pt")




#####################################################################################################
# Step 5: Evaluate the accuracy of your network on the test dataset
#--------------------------------------------------------------------
# Use the above trained model and predict labels for your test data - this is nothing but the
# forward pass.
# Check accuracy by comparing with the ground truth labels and report it.
# What are the cases where the accuracy is low?
# Can you visualize these cases?

# Note: If you are loading a saved model then you must call model.eval() before you do inference.
# model = torch.load(PATH)
# model.eval()
#####################################################################################################

#####################################################################################################
# Step 6: Evaluate the accuracy of your network on your own data
#--------------------------------------------------------------------
# Use images that are not part of the training dataset.
# We have provided a larger dataset from which you can pick images.
# What does the network predict and why?
#####################################################################################################
