# PyTorch Setup
## 前置
Anaconda 的安装  
环境管理：
`conda create -n python=(版本号)`   
环境激活  
`conda activate pytorch`
## 安装
`pip3 install torch torchvision`  
检查GPU可用  
`import torch`  
`print(torch.backends.mps.is_available())`  
`print(torch.backends.mps.is_built())`  
--返回True  
安装PyCharm