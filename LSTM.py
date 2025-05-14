# practice 2025/3/21 20:35
import torch
import torch.nn as nn
import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler

# 参数设置
SEQ_LENGTH = 10  # 输入序列长度
HIDDEN_SIZE = 32  # LSTM隐藏层大小
NUM_LAYERS = 2  # LSTM层数
LEARNING_RATE = 0.001
EPOCHS = 100
BATCH_SIZE = 16


# 生成示例数据（正弦波 + 随机噪声）
def generate_data(seq_length, n_samples=1000):
    time = np.linspace(0, 20 * np.pi, n_samples)
    data = np.sin(time) + 0.1 * np.random.randn(n_samples)
    return data.reshape(-1, 1)


# 数据预处理
def create_sequences(data, seq_length):
    X, y = [], []
    for i in range(len(data) - seq_length - 1):
        X.append(data[i:i + seq_length])
        y.append(data[i + seq_length])
    return np.array(X), np.array(y)


# 数据准备
raw_data = generate_data(SEQ_LENGTH)
scaler = MinMaxScaler(feature_range=(-1, 1))
scaled_data = scaler.fit_transform(raw_data)

X, y = create_sequences(scaled_data, SEQ_LENGTH)
split = int(0.8 * len(X))
X_train, X_test = X[:split], X[split:]
y_train, y_test = y[:split], y[split:]

# 转换为PyTorch张量
train_data = torch.utils.data.TensorDataset(
    torch.from_numpy(X_train).float(),
    torch.from_numpy(y_train).float()
)
test_data = torch.utils.data.TensorDataset(
    torch.from_numpy(X_test).float(),
    torch.from_numpy(y_test).float()
)

train_loader = torch.utils.data.DataLoader(
    train_data, batch_size=BATCH_SIZE, shuffle=True)
test_loader = torch.utils.data.DataLoader(
    test_data, batch_size=BATCH_SIZE, shuffle=False)


# LSTM模型定义
class LSTMModel(nn.Module):
    def __init__(self, input_size=1, hidden_size=32, num_layers=2, output_size=1):
        super().__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=0.2
        )
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)

        out, _ = self.lstm(x, (h0, c0))
        out = self.fc(out[:, -1, :])
        return out


# 初始化模型
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = LSTMModel(hidden_size=HIDDEN_SIZE, num_layers=NUM_LAYERS).to(device)
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)


# 训练函数
def train(model, train_loader, epochs):
    model.train()
    for epoch in range(epochs):
        total_loss = 0
        for batch_x, batch_y in train_loader:
            batch_x, batch_y = batch_x.to(device), batch_y.to(device)

            optimizer.zero_grad()
            outputs = model(batch_x)
            loss = criterion(outputs, batch_y)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 5)  # 梯度裁剪
            optimizer.step()

            total_loss += loss.item()

        avg_loss = total_loss / len(train_loader)
        if (epoch + 1) % 10 == 0:
            print(f'Epoch [{epoch + 1}/{epochs}], Loss: {avg_loss:.4f}')


# 评估函数
def evaluate(model, test_loader):
    model.eval()
    total_loss = 0
    with torch.no_grad():
        for batch_x, batch_y in test_loader:
            batch_x, batch_y = batch_x.to(device), batch_y.to(device)
            outputs = model(batch_x)
            total_loss += criterion(outputs, batch_y).item()
    return total_loss / len(test_loader)


# 开始训练
train(model, train_loader, EPOCHS)

# 评估模型
test_loss = evaluate(model, test_loader)
print(f'Test Loss: {test_loss:.4f}')

# 预测示例
model.eval()
with torch.no_grad():
    test_input = X_test[0].reshape(1, SEQ_LENGTH, 1)  # 取第一个测试样本
    test_input = torch.from_numpy(test_input).float().to(device)
    prediction = model(test_input).cpu().numpy()

# 反归一化
true_value = scaler.inverse_transform(y_test[0].reshape(-1, 1))
predicted_value = scaler.inverse_transform(prediction)

print(f'True value: {true_value[0][0]:.4f}, Predicted: {predicted_value[0][0]:.4f}')

# 可视化预测结果
plt.figure(figsize=(12, 6))
plt.plot(scaler.inverse_transform(scaled_data), label='Original Data')
plt.plot(range(split + SEQ_LENGTH, len(scaled_data)),
         scaler.inverse_transform(model(torch.from_numpy(X).float().to(device)).cpu().numpy()),
         label='Predicted')
plt.axvline(x=split, color='r', linestyle='--', label='Train/Test Split')
plt.legend()
plt.show()