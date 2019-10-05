---
title: Neural Network Overview
date: 12 Aug 2019
---

## Neural Netowrk (NN) Types

### Convolutional NN (CNN)

The heart of this is convolution:

![](conv.gif)

- CNNs are feedforward systems, meaning, info is fed from one layer to the next
- CNNs only consider the current inputs
- Layers:
    - *Convolution layer:* most computational heavy lifting layer
    - *Pooling layer:* sandwiched aroun convolutional layers to reduce the size
    of what the cconvolutional layers produce. This helps to reduce memory
    requirements
    - *Normalizing layer:* converts all inputs to a value with zero mean and a
    standard deviation of 1. This improves performance and stability of the NN.
    There are many different normalizing layer topologies like ReLU.
- Common uses: object identification in computer vision

### Recursive NN (RNN)

- Similar to CNN, but RNNs have a feedback capability
- RNNs concider both the current and previous inputs
    - can predicct what comes next because it learns order
- Common uses: stock prediction, trading stock, language, or anyting that
follows a pattner

## Neural Networks (NN) Libraries

- **You Only Look Once(YOLO):** A single NN that looks a the whole image by
dividing it up into regions and predicts bounding boxes and probabilities for
each region. It is extremely fast.
    - Yolov3-tiny is a smaller version for constrained environments (embedded
        systems)
- **Tensorflow:** came from Google research and still used/backed by them.
    - Python and C++
- **Open Neural Network eXchange (ONNX):** Large consordium of companies supporting
tools/architecture where models are trained in one framework and transfered
to another framework.
    - Supports: Caffe2, CNTK, MXNet, pyTorch, Tensorflow
    - Python and C++
- **Theano:** as of 2019, not funded or maintained anymore
- **Microsoft Cognitive Toolkit (CNTK):** not maintained anymore after version
2.7 based on disclaimer on github site. They suggest using ONNX
- **Tencent/ncnn:** A NN for cellphone/embedded systems
    - [github repo](https://github.com/Tencent/ncnn)

## Tools

- **Keras:** Highlevel NN tools written in python
    - Supports: Tensorflow, CNTK or Theano as the backend NN
    - Also funded by Google and large market support
        - Apple supports CoreML integration for iOS
    - Adopted by CERN and NASA
- **pyTorch:** Highlevel NN tools written in python and C++
    - Supports CUDA (NVIDIA GPU)

## Pre-trained

- **ResNet50:** a 50 layer, pretrained CNN on 1 million images that can
catagorize 1000 objects
    - image size: 224x224 px

# References

- [Deep learning on Raspberry Pi](https://medium.com/nanonets/how-to-easily-detect-objects-with-deep-learning-on-raspberrypi-225f29635c74)
- [ResNet Tensorflow example](https://github.com/taki0112/ResNet-Tensorflow)
- [Tensorflow Cookbook](https://github.com/taki0112/Tensorflow-Cookbook)
