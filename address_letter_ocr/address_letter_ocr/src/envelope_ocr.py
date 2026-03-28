"""
Envelope Address OCR Recognizer
Extract the first letter of recipient name from TO address in envelope images
"""

import os
import re
from pathlib import Path
from typing import Optional, Tuple

import pytesseract
from PIL import Image
import cv2
import numpy as np


class EnvelopeOCR:
    """Envelope OCR Recognition Class"""
    
    # Supported image formats
    SUPPORTED_FORMATS = {'.png', '.jpg', '.jpeg', '.bmp', '.tiff', '.gif'}
    
    def __init__(self, tesseract_cmd: Optional[str] = None):
        """
        Initialize OCR recognizer
        
        Args:
            tesseract_cmd: Path to Tesseract executable (optional)
        """
        if tesseract_cmd:
            pytesseract.pytesseract.tesseract_cmd = tesseract_cmd
    
    def preprocess_image(self, image_path: str) -> np.ndarray:
        """
        Preprocess image to improve OCR accuracy
        
        Args:
            image_path: Path to image file
            
        Returns:
            Preprocessed image array
        """
        # Read image
        img = cv2.imread(image_path)
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        
        # Apply binary thresholding
        _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        return binary
    
    def extract_text(self, image_path: str) -> str:
        """
        Extract text from image
        
        Args:
            image_path: Path to image file
            
        Returns:
            Recognized text
        """
        # Preprocess image
        processed_img = self.preprocess_image(image_path)
        
        # Use Tesseract for OCR recognition
        # Config: --psm 6 assumes a single uniform block of text
        config = '--psm 6 -l eng'
        text = pytesseract.image_to_string(processed_img, config=config)
        
        return text
    
    def find_to_address(self, text: str) -> Optional[str]:
        """
        Find TO address section from text
        
        Args:
            text: Full text from OCR recognition
            
        Returns:
            Recipient name from TO address, or None if not found
        """
        # Multiple patterns to find TO address
        patterns = [
            # Match content after "TO:"
            r'TO:\s*\n?\s*([A-Za-z][A-Za-z\-\s\.]+)',
            # Match content after "To:"
            r'To:\s*\n?\s*([A-Za-z][A-Za-z\-\s\.]+)',
            # Match content after "TO" (without colon)
            r'TO\s+([A-Za-z][A-Za-z\-\s\.]+)',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, text, re.IGNORECASE | re.MULTILINE)
            if match:
                name = match.group(1).strip()
                # Only take the first line (recipient name)
                name = name.split('\n')[0].strip()
                return name
        
        return None
    
    def get_first_letter(self, name: str) -> Optional[str]:
        """
        Get the first letter of the name
        
        Args:
            name: Recipient name
            
        Returns:
            First letter (uppercase), or None if cannot extract
        """
        if not name:
            return None
        
        # Remove leading spaces and special characters, find first letter
        for char in name:
            if char.isalpha():
                return char.upper()
        
        return None
    
    def process_image(self, image_path: str) -> Tuple[Optional[str], Optional[str], str]:
        """
        Process a single envelope image
        
        Args:
            image_path: Path to image file
            
        Returns:
            Tuple of (first_letter, recipient_name, full_text)
        """
        # Extract text
        full_text = self.extract_text(image_path)
        
        # Find TO address
        recipient_name = self.find_to_address(full_text)
        
        # Get first letter
        first_letter = self.get_first_letter(recipient_name) if recipient_name else None
        
        return first_letter, recipient_name, full_text
    
    def process_directory(self, directory: str) -> dict:
        """
        Process all envelope images in a directory
        
        Args:
            directory: Path to image directory
            
        Returns:
            Dictionary containing processing results for each image
        """
        results = {}
        dir_path = Path(directory)
        
        if not dir_path.exists():
            raise FileNotFoundError(f"Directory not found: {directory}")
        
        # Iterate through all image files in directory
        for file_path in sorted(dir_path.iterdir()):
            if file_path.suffix.lower() in self.SUPPORTED_FORMATS:
                try:
                    first_letter, recipient_name, full_text = self.process_image(str(file_path))
                    results[file_path.name] = {
                        'first_letter': first_letter,
                        'recipient_name': recipient_name,
                        'full_text': full_text,
                        'status': 'success' if first_letter else 'no_to_address_found'
                    }
                except Exception as e:
                    results[file_path.name] = {
                        'first_letter': None,
                        'recipient_name': None,
                        'full_text': None,
                        'status': f'error: {str(e)}'
                    }
        
        return results


def print_results(results: dict) -> None:
    """
    Print processing results
    
    Args:
        results: Result dictionary returned by process_directory
    """
    print("\n" + "=" * 70)
    print("Envelope Address OCR Recognition Results")
    print("=" * 70)
    
    success_count = 0
    total_count = len(results)
    
    for filename, data in results.items():
        print(f"\n📧 File: {filename}")
        print("-" * 50)
        
        if data['first_letter']:
            print(f"   ✅ TO Address First Letter: {data['first_letter']}")
            print(f"   👤 Recipient Name: {data['recipient_name']}")
            success_count += 1
        else:
            print(f"   ❌ Status: {data['status']}")
            if data['full_text']:
                print(f"   📝 Full Recognized Text:\n{data['full_text'][:200]}...")
    
    print("\n" + "=" * 70)
    print(f"📊 Statistics: Successfully recognized {success_count}/{total_count} images")
    print("=" * 70)
