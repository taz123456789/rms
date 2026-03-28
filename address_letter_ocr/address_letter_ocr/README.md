# 信封地址OCR识别项目

识别信封图片中 **TO** 地址收件人姓名的**第一个字母**。

## 📦 安装依赖

### 1. 安装 Tesseract OCR

**macOS:**
```bash
brew install tesseract
```

**Ubuntu/Debian:**
```bash
sudo apt-get install tesseract-ocr
```

**Windows:**
从 [Tesseract 官方发布页](https://github.com/UB-Mannheim/tesseract/wiki) 下载安装。

### 2. 安装 Python 依赖

```bash
cd /Users/user/Documents/study/address_letter_ocr
pip install -r requirements.txt
```

## 🚀 使用方法

### 基本用法

```bash
# 识别 pictures 目录下的所有信封图片
python main.py
```

### 指定目录

```bash
python main.py -d /path/to/your/images
```

### 显示详细信息

```bash
python main.py -v
```

### 查看帮助

```bash
python main.py -h
```

## 📁 项目结构

```
address_letter_ocr/
├── main.py                 # 主程序入口
├── requirements.txt        # Python依赖
├── README.md              # 项目说明
├── src/
│   ├── __init__.py
│   └── envelope_ocr.py    # OCR核心代码
└── pictures/              # 信封图片目录
    ├── Letter_F.png
    ├── Letter-S.png
    └── ...
```

## 📋 输出示例

```
🔍 正在扫描目录: /Users/user/Documents/study/address_letter_ocr/pictures
📁 查找信封图片...

======================================================================
信封地址OCR识别结果
======================================================================

📧 文件: Letter-D.png
--------------------------------------------------
   ✅ TO地址首字母: D
   👤 收件人姓名: Dixie-Rae Neli Ebner

📧 文件: Letter_F.png
--------------------------------------------------
   ✅ TO地址首字母: F
   👤 收件人姓名: Fabiano Barun Goldhirsch

======================================================================
📊 统计: 成功识别 12/12 张图片
======================================================================

📝 首字母汇总:
   Letter-D.png: D
   Letter_F.png: F
   ...
```

## 🔧 技术说明

- 使用 **Tesseract OCR** 进行文字识别
- 使用 **OpenCV** 进行图像预处理（灰度化、二值化）
- 使用正则表达式匹配 "TO:" 格式的地址
- 支持 PNG、JPG、JPEG、BMP、TIFF、GIF 格式的图片

## ⚠️ 注意事项

1. 确保信封图片清晰，"TO:" 地址部分可见
2. 如果识别率不高，可以尝试提高图片分辨率
3. Tesseract 需要正确安装并添加到系统 PATH
