#!/usr/bin/env python3
"""
Envelope Address OCR Main Program
Extract the first letter of recipient name from TO address in all envelope images
"""

import argparse
import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from envelope_ocr import EnvelopeOCR, print_results


def main():
    """Main function"""
    parser = argparse.ArgumentParser(
        description='Extract the first letter of recipient name from TO address in envelope images',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python main.py                           # Use default pictures directory
  python main.py -d /path/to/images        # Specify image directory
  python main.py -v                         # Show verbose output
        '''
    )
    
    parser.add_argument(
        '-d', '--directory',
        type=str,
        default=None,
        help='Image directory path (default: ./pictures)'
    )
    
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Show detailed OCR recognized text'
    )
    
    parser.add_argument(
        '-t', '--tesseract',
        type=str,
        default=None,
        help='Path to Tesseract executable'
    )
    
    args = parser.parse_args()
    
    # Determine image directory
    if args.directory:
        pictures_dir = Path(args.directory)
    else:
        # Default to pictures folder under project root
        pictures_dir = Path(__file__).parent / 'pictures'
    
    if not pictures_dir.exists():
        print(f"❌ Error: Directory not found - {pictures_dir}")
        sys.exit(1)
    
    print(f"🔍 Scanning directory: {pictures_dir}")
    print(f"📁 Looking for envelope images...")
    
    # Initialize OCR recognizer
    try:
        ocr = EnvelopeOCR(tesseract_cmd=args.tesseract)
    except Exception as e:
        print(f"❌ Failed to initialize OCR: {e}")
        print("\n💡 Tip: Please make sure Tesseract OCR is installed:")
        print("   macOS: brew install tesseract")
        print("   Ubuntu: sudo apt-get install tesseract-ocr")
        print("   Windows: Download from https://github.com/UB-Mannheim/tesseract/wiki")
        sys.exit(1)
    
    # Process all images in directory
    try:
        results = ocr.process_directory(str(pictures_dir))
    except FileNotFoundError as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error processing images: {e}")
        sys.exit(1)
    
    if not results:
        print("⚠️ No image files found")
        sys.exit(0)
    
    # Print results
    print_results(results)
    
    # Show full OCR text if verbose mode is enabled
    if args.verbose:
        print("\n" + "=" * 70)
        print("Detailed OCR Recognized Text")
        print("=" * 70)
        for filename, data in results.items():
            print(f"\n--- {filename} ---")
            print(data.get('full_text', 'N/A'))
    
    # Output first letter summary
    print("\n📝 First Letter Summary:")
    letters = []
    for filename, data in sorted(results.items()):
        letter = data.get('first_letter', '?')
        letters.append(f"{filename}: {letter if letter else '?'}")
    
    for item in letters:
        print(f"   {item}")
    
    # Return success/failure status
    success_count = sum(1 for d in results.values() if d.get('first_letter'))
    return 0 if success_count == len(results) else 1


if __name__ == '__main__':
    sys.exit(main())
