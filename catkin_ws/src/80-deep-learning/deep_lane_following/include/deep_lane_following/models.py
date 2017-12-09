import torch
import torch.nn as nn
import torch.nn.functional as F

class CNNPolicy(nn.Module):
    def __init__(self, num_inputs):
        super(CNNPolicy, self).__init__()
        self.bn0 = nn.BatchNorm2d(3)
        self.conv1 = nn.Conv2d(num_inputs, 32, 8, stride=4)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 64, 4, stride=2)
        self.bn2 = nn.BatchNorm2d(64)
        self.conv3 = nn.Conv2d(64, 32, 3, stride=1)
        self.bn3 = nn.BatchNorm2d(32)

        self.linear1 = nn.Linear(32 * 4 * 4, 512)
        self.linear2 = nn.Linear(512, 2)

        self.reset_parameters()

    def reset_parameters(self):
        def weights_init(m):
            classname = m.__class__.__name__
            if classname.find('Conv') != -1 or classname.find('Linear') != -1:
                nn.init.orthogonal(m.weight.data)
                if m.bias is not None:
                    m.bias.data.fill_(0)
        self.apply(weights_init)

        relu_gain = nn.init.calculate_gain('relu')
        self.conv1.weight.data.mul_(relu_gain)
        self.conv2.weight.data.mul_(relu_gain)
        self.conv3.weight.data.mul_(relu_gain)
        self.linear1.weight.data.mul_(relu_gain)

    def forward(self, inputs):
        x = self.bn1(self.conv1(self.bn0(inputs)))
        x = F.relu(x)

        x = self.bn2(self.conv2(x))
        x = F.relu(x)

        x = self.bn3(self.conv3(x))
        x = F.relu(x)

        x = x.view(-1, 32 * 4 * 4)
        x = self.linear1(x)
        x = F.dropout(F.relu(x), 0.5, self.training)
        x = self.linear2(x)

        x = x + torch.autograd.Variable(torch.Tensor([1.12, 0]))

        return x

class CNNPolicy2(nn.Module):
    def __init__(self, num_inputs):
        super(CNNPolicy2, self).__init__()
        self.bn0 = nn.BatchNorm2d(3)
        self.conv1 = nn.Conv2d(num_inputs, 32, 3, stride=2)
        self.bn1 = nn.BatchNorm2d(32)
        self.conv2 = nn.Conv2d(32, 32, 3, stride=2)
        self.bn2 = nn.BatchNorm2d(32)
        self.conv3 = nn.Conv2d(32, 32, 3, stride=1)
        self.bn3 = nn.BatchNorm2d(32)
        self.conv4 = nn.Conv2d(32, 32, 3, stride=1)
        self.bn4 = nn.BatchNorm2d(32)
        self.conv5 = nn.Conv2d(32, 32, 3, stride=1)
        self.bn5 = nn.BatchNorm2d(32)

        self.linear1 = nn.Linear(32 * 9 * 9, 512)
        self.linear2 = nn.Linear(512, 2)

        self.reset_parameters()

    def reset_parameters(self):
        def weights_init(m):
            classname = m.__class__.__name__
            if classname.find('Conv') != -1 or classname.find('Linear') != -1:
                nn.init.orthogonal(m.weight.data)
                if m.bias is not None:
                    m.bias.data.fill_(0)
        self.apply(weights_init)

        relu_gain = nn.init.calculate_gain('relu')
        self.conv1.weight.data.mul_(relu_gain)
        self.conv2.weight.data.mul_(relu_gain)
        self.conv3.weight.data.mul_(relu_gain)
        self.conv4.weight.data.mul_(relu_gain)
        self.conv5.weight.data.mul_(relu_gain)
        self.linear1.weight.data.mul_(relu_gain)

    def forward(self, inputs):
        x = self.bn1(self.conv1(self.bn0(inputs)))
        x = F.relu(x)

        x = self.bn2(self.conv2(x))
        x = F.relu(x)

        x = self.bn3(self.conv3(x))
        x = F.relu(x)

        x = self.bn4(self.conv4(x))
        x = F.relu(x)

        x = self.bn5(self.conv5(x))
        x = F.relu(x)

        x = x.view(-1, 32 * 9 * 9)
        x = self.linear1(x)
        x = F.dropout(F.relu(x), 0.5, self.training)
        x = self.linear2(x)

        x = x + torch.autograd.Variable(torch.Tensor([1.12, 0]))

        return x
