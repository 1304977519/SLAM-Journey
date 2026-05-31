import torch
import numpy as np
import matplotlib.pyplot as plt
from torch.utils.data import DataLoader
from torchvision import datasets
from torchvision.transforms import v2

data = [[1, 2], [3, 4]]
x_data = torch.tensor(data)
print(x_data)
print()

np_array = np.array(data)
x_np = torch.from_numpy(np_array)
print(x_np)
print()

#创建一个和 x_data 形状一样的 Tensor，但是里面全部填 1
x_ones = torch.ones_like(x_data)
#创建一个和 x_data 形状一样的 Tensor，[0, 1) 范围内的随机数
x_rand = torch.rand_like(x_data, dtype=torch.float)
print(x_ones)
print(x_rand)
print()

shape = (2, 3)
#[0, 1)
rand_tensor = torch.rand(shape)
ones_tensor = torch.ones(shape)
zeros_tensor = torch.zeros(shape)
print(rand_tensor)
print(ones_tensor)
print(zeros_tensor)
print()

tensor = torch.rand(3, 4)
print(f"Shape of tensor: {tensor.shape}")
print(f"Datatype of tensor: {tensor.dtype}")
print(f"Device tensor is stored on: {tensor.device}")
if torch.cuda.is_available():
    tensor = tensor.to("cuda")
    print(f"After moving to GPU: {tensor.device}")
else:
    print("CUDA 不可用，当前 tensor 仍在 CPU 上。")
print()

tensor = torch.ones(4, 4)
print("原始 tensor:")
print(tensor)
print("\n第一行:")
print(tensor[0])
print("\n第一列:")
print(tensor[:, 0])
print("\n最后一列:")
print(tensor[..., -1])
tensor[:, 1] = 0
print("\n把第二列改成 0 后:")
print(tensor)
t1 = torch.cat([tensor, tensor, tensor], dim=1)
print("\n按列拼接后的 t1:")
print(t1)
t2 = torch.cat([tensor, tensor, tensor], dim=0)
print("\n按列拼接后的 t2:")
print(t2)
print("\ntensor 矩阵乘法 tensor.T:")
print(tensor @ tensor.T)
print("\ntensor 逐元素相乘:")
print(tensor * tensor)
print()

training_data = datasets.FashionMNIST(
    root="data",
    train=True,
    download=True,
    transform=v2.Compose([
        v2.ToImage(),
        v2.ToDtype(torch.float32, scale=True)
    ])
)
test_data = datasets.FashionMNIST(
    root="data",
    train=False,
    download=True,
    transform=v2.Compose([
        v2.ToImage(),
        #先转到float
        #0   -> 0.0
        #128 -> 0.5019 左右
        #255 -> 1.0
        v2.ToDtype(torch.float32, scale=True)
    ])
)
print(f"训练集样本数量: {len(training_data)}")
print(f"测试集样本数量: {len(test_data)}")
#这里的数据是(img, label)
img, label = training_data[0]
print(f"单个样本 image shape: {img.shape}")
print(f"单个样本 label: {label}")
print()


train_dataloader = DataLoader(
    training_data,
    batch_size=64,
    #每一轮训练前把数据顺序打乱
    shuffle=True
)
test_dataloader = DataLoader(
    test_data,
    batch_size=64,
    shuffle=True
)
#他输出的应该是(train_features, train_labels)
train_features, train_labels = next(iter(train_dataloader))
print(f"Feature batch shape: {train_features.size()}")
print(f"Labels batch shape: {train_labels.size()}")
#把大小为 1 的维度删掉
img = train_features[0].squeeze()
label = train_labels[0]
print(f"第一个 batch 中第一张图的 label: {label}")
print()

#黑白
plt.imshow(img, cmap="gray")
plt.title(f"Label: {label}")
plt.axis("off")
plt.show()