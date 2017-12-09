from models import CNNPolicy, CNNPolicy2
import torch
import os

root = os.path.dirname(__file__)
path = os.path.join(root, "model.pth.tar")

def load_model():
    model = CNNPolicy2(3)
    print("=> loading checkpoint '{}'".format(path))
    checkpoint = torch.load(path)
    model.load_state_dict(checkpoint['state_dict'])
    model.eval()
    return model

