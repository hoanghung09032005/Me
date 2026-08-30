"""
train.py
--------
CHẠY TÁCH BIỆT, MỘT LẦN, KHÔNG LIÊN QUAN đến GUI hay xe thật.
Đọc dataset đã ghi từ mode4_ai.py (thư mục config.AI_DATASET_DIR), train
LineErrorNet, lưu ra config.AI_MODEL_PATH để mode4_ai.py load lại.

Cách chạy:
    cd robot_car_project
    python3 train.py
"""

import os
import csv
import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader, random_split

import config
from ai_model import LineErrorNet, preprocess_frame


class LineDataset(Dataset):
    def __init__(self, dataset_dir):
        self.dataset_dir = dataset_dir
        self.samples = []
        label_path = os.path.join(dataset_dir, "labels.csv")
        with open(label_path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.samples.append((row["file"], float(row["error"])))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        filename, error = self.samples[idx]
        img_path = os.path.join(self.dataset_dir, filename)
        bgr = cv2.imread(img_path)
        chw = preprocess_frame(bgr, config.AI_LINE_CROP_TOP)
        return torch.from_numpy(chw), torch.tensor(np.float32(error))


def train():
    dataset = LineDataset(config.AI_DATASET_DIR)
    print(f"Tổng số mẫu: {len(dataset)}")
    if len(dataset) < 200:
        print("CẢNH BÁO: dataset còn khá ít (<200 ảnh). Nên thu thập thêm "
              "trước khi train để tránh overfitting.")

    val_size = max(1, int(0.2 * len(dataset)))
    train_size = len(dataset) - val_size
    train_ds, val_ds = random_split(dataset, [train_size, val_size])

    train_loader = DataLoader(train_ds, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=32)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Train trên: {device}")

    model = LineErrorNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)
    criterion = nn.MSELoss()

    best_val_loss = float("inf")
    epochs = 40

    for epoch in range(epochs):
        model.train()
        train_loss = 0.0
        for imgs, errors in train_loader:
            imgs, errors = imgs.to(device), errors.to(device)
            optimizer.zero_grad()
            preds = model(imgs)
            loss = criterion(preds, errors)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * imgs.size(0)
        train_loss /= len(train_ds)

        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for imgs, errors in val_loader:
                imgs, errors = imgs.to(device), errors.to(device)
                preds = model(imgs)
                val_loss += criterion(preds, errors).item() * imgs.size(0)
        val_loss /= len(val_ds)

        print(f"Epoch {epoch+1:2d}/{epochs} - train_loss={train_loss:.4f}  val_loss={val_loss:.4f}")

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), config.AI_MODEL_PATH)
            print(f"  -> Lưu model tốt nhất vào {config.AI_MODEL_PATH}")

    print("Train xong. val_loss tốt nhất:", best_val_loss)


if __name__ == "__main__":
    train()