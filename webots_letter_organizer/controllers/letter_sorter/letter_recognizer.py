import os
import re
import subprocess
import tempfile

import cv2
import numpy as np
from scipy.ndimage import uniform_filter, label as scipy_label
import pytesseract




class LetterRecognizer:
    """
    Recognize letters (A/B/C/D) from camera images using Tesseract OCR.

    Interface is identical to the previous template-matching version:
        recognize(image, confidence_threshold) -> (letter, confidence)
    """

    VALID_LETTERS = {"A", "B", "C", "D"}

    NAME_MAP = {
        "ALICE": "A", "BOB": "B", "CHARLIE": "C", "DAVID": "D",
    }

    # Broad patterns: first char = T/t/l/1/I/7, second char = o/O/a/A/0/c/e
    # This covers To, TO, Ta, TA, Tc, Te, lo, 1o, Io, 7o, T0, etc.
    TO_PATTERNS = [
        # Standard "To:" / "TO:" with colon or space
        r'[Ttl1I7][oOaA0cCeE][:\s]+\s*([A-Za-z][A-Za-z\-\s\.]+)',
        # Without colon, just space before name
        r'[Ttl1I7][oOaA0cCeE]\s+([A-Za-z][A-Za-z\-\s\.]+)',
    ]

    # Pre-matching normalizations: fix common OCR misreads of "To:" back
    # to a canonical form so the regex can match reliably.
    # Each tuple is (compiled_regex, replacement_string).
    _OCR_FIXES = None  # lazily compiled

    @classmethod
    def _get_ocr_fixes(cls):
        if cls._OCR_FIXES is None:
            raw = [
                # "Ta:" / "TA:" -> "To:"
                (r'\bTa\s*:', 'To:'), (r'\bTA\s*:', 'To:'), (r'\bta\s*:', 'To:'),
                # "Tc:" / "Te:" -> "To:"
                (r'\bTc\s*:', 'To:'), (r'\bTe\s*:', 'To:'), (r'\btc\s*:', 'To:'),
                (r'\bte\s*:', 'To:'),
                # "lo:" / "Lo:" (l misread as T) -> "To:"
                (r'\blo\s*:', 'To:'), (r'\bLo\s*:', 'To:'),
                # "1o:" / "Io:" / "io:" -> "To:"
                (r'\b1o\s*:', 'To:'), (r'\bIo\s*:', 'To:'), (r'\bio\s*:', 'To:'),
                # "7o:" / "7O:" -> "To:"
                (r'\b7o\s*:', 'To:'), (r'\b7O\s*:', 'To:'),
                # "T0:" (zero) -> "To:"
                (r'\bT0\s*:', 'To:'), (r'\bt0\s*:', 'To:'),
                # "fo:" / "Fo:" -> "To:"
                (r'\bfo\s*:', 'To:'), (r'\bFo\s*:', 'To:'),
                # "|" at start often is border artifact
                (r'^\|\s*', ''),
            ]
            cls._OCR_FIXES = [(re.compile(p), r) for p, r in raw]
        return cls._OCR_FIXES

    def __init__(self, template_dir=None, tesseract_cmd=None):

        global _TESSERACT_CMD
        self.template_dir = template_dir

        if tesseract_cmd:
            _TESSERACT_CMD = tesseract_cmd

            pytesseract.pytesseract.tesseract_cmd = tesseract_cmd


            backend = "pytesseract"
            print("  [Recognizer] Tesseract OCR ready (backend: {})".format(backend))



    def _find_envelope_region(self, arr):
        """
        Locate the envelope (largest bright connected component) in a
        3D-rendered scene image.

        Args:
            arr: RGB numpy array (H, W, 3)

        Returns:
            (row_min, row_max, col_min, col_max) or None
        """


        gray = np.mean(arr, axis=2)
        bright_mask = gray > 100
        labeled_arr, num_features = scipy_label(bright_mask)

        best = None
        best_size = 0
        for i in range(1, num_features + 1):
            region = (labeled_arr == i)
            size = int(region.sum())
            if size > best_size:
                best_size = size
                rows, cols = np.where(region)
                best = (int(rows.min()), int(rows.max()),
                        int(cols.min()), int(cols.max()))
        return best

    def _adaptive_binarize(self, envelope_arr, kernel=20, offset=8):
        """
        Adaptive threshold binarization: text pixels are darker than
        their local mean -> marked white.

        Uses the red channel which gives best contrast for the blue-sky
        background typical of Webots scenes.

        Args:
            envelope_arr: RGB numpy array of envelope region
            kernel: local mean window size
            offset: how much darker than local mean to qualify as text

        Returns:
            Binary uint8 array (0/255), text pixels = 255
        """


        gray = envelope_arr[:, :, 0].astype(float)  # red channel
        local_mean = uniform_filter(gray, size=kernel)
        text_mask = (gray < local_mean - offset).astype(np.uint8) * 255
        return text_mask

    def _crop_text_region(self, binary_img):
        arr = binary_img
        if len(arr.shape) == 3:
            arr = arr[:, :, 0]

        h, w = arr.shape
        if h < 20 or w < 20:
            return binary_img

        dark = (arr < 128).astype(np.uint8)

        # -- Count dark/light transitions per row --
        TRANS_THRESHOLD = 16
        transitions = np.zeros(h, dtype=int)
        for r in range(h):
            diffs = np.abs(np.diff(dark[r, :].astype(np.int8)))
            transitions[r] = int(np.sum(diffs))

        text_rows = np.where(transitions >= TRANS_THRESHOLD)[0]
        if len(text_rows) == 0:
            # Lower threshold and retry
            TRANS_THRESHOLD = 10
            text_rows = np.where(transitions >= TRANS_THRESHOLD)[0]
        if len(text_rows) == 0:
            return binary_img

        # -- Cluster contiguous text rows --
        GAP_LIMIT = 30
        groups = []
        current_group = [int(text_rows[0])]
        for i in range(1, len(text_rows)):
            if text_rows[i] - text_rows[i - 1] <= GAP_LIMIT:
                current_group.append(int(text_rows[i]))
            else:
                groups.append(current_group)
                current_group = [int(text_rows[i])]
        groups.append(current_group)

        # Pick the largest cluster
        best_group = max(groups, key=len)

        # -- Column extent within the text band --
        pad = 40
        r_start = max(0, best_group[0] - pad)
        r_end = min(h, best_group[-1] + pad)

        text_strip = dark[r_start:r_end, :]
        col_dark = np.sum(text_strip, axis=0)
        text_cols = np.where(col_dark > 1)[0]

        if len(text_cols) == 0:
            c_start, c_end = 0, w
        else:
            c_start = max(0, int(text_cols[0]) - pad)
            c_end = min(w, int(text_cols[-1]) + pad)

        return binary_img[r_start:r_end, c_start:c_end]


    def _preprocess_adaptive(self, image):
        # Convert BGR -> RGB for consistent channel order
        if len(image.shape) == 3 and image.shape[2] == 3:
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            rgb = image

        # 1. Locate envelope region
        bbox = self._find_envelope_region(rgb)
        if bbox:
            r1, r2, c1, c2 = bbox
            pad_r = int((r2 - r1) * 0.6)
            pad_c = int((c2 - c1) * 0.4)
            r1 = max(0, r1 - 10)
            r2 = min(rgb.shape[0], r2 + pad_r)
            c1 = max(0, c1 - pad_c)
            c2 = min(rgb.shape[1], c2 + pad_c)
            envelope = rgb[r1:r2, c1:c2]
        else:
            envelope = rgb

        # 2. Adaptive binarization
        binary = self._adaptive_binarize(envelope)

        # 3. 4x upscale (nearest neighbor to preserve edges)
        h, w = binary.shape[:2]
        big = cv2.resize(binary, (w * 4, h * 4),
                         interpolation=cv2.INTER_NEAREST)

        # 4. Rotate -90 deg (correct Webots camera orientation)
        rotated = cv2.rotate(big, cv2.ROTATE_90_CLOCKWISE)

        # 5. Invert -> white background, black text
        inverted = cv2.bitwise_not(rotated)

        # 6. Crop to text band using transition-count detection
        cropped = self._crop_text_region(inverted)

        return cropped

    # ------------------------------------------------------------------
    # Legacy preprocessing: simple Otsu (for clean texture images)
    # ------------------------------------------------------------------

    def _preprocess_simple(self, image):
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()

        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        _, binary = cv2.threshold(blurred, 0, 255,
                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary


    def _preprocess(self, image):
        mean_brightness = np.mean(image)
        if mean_brightness < 150:
            return self._preprocess_adaptive(image)
        else:
            return self._preprocess_simple(image)

    def _extract_text_pytesseract(self, image, psm=6):
        """Run OCR via pytesseract Python binding."""
        config = "--psm {} -l eng".format(psm)
        text = pytesseract.image_to_string(image, config=config)
        return text

    def _extract_text_cli(self, image, psm=6):
        """Run OCR via tesseract CLI subprocess."""
        if _TESSERACT_CMD is None:
            return ""

        script_dir = os.path.dirname(os.path.abspath(__file__))
        fd, tmp_path = tempfile.mkstemp(suffix=".png", dir=script_dir)
        os.close(fd)

        try:
            cv2.imwrite(tmp_path, image)
            result = subprocess.run(
                [_TESSERACT_CMD, tmp_path, "stdout", "--psm", str(psm)],
                capture_output=True,
                timeout=30,
            )
            text = result.stdout.decode("utf-8", errors="replace").strip()
            return text
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return ""
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)

    def _extract_text(self, image):



        ocr_fn = self._extract_text_pytesseract


        candidates = []
        for psm in [7, 6, 8, 13]:
            try:
                text = ocr_fn(image, psm=psm)
                if text.strip():
                    candidates.append(text)
            except Exception:
                continue

        if not candidates:
            return ""

        # Score each candidate: prefer text with known names
        known_names = list(self.NAME_MAP.keys())

        def _score(t):
            tu = t.upper()
            # Has known name? Best.
            for n in known_names:
                if n in tu:
                    return (3, len(t))
            # Has "To" variant? Good.
            if re.search(r'[Ttl1I7][oOaA0cCeE]\s*:', t):
                return (2, len(t))
            # Fallback: longer is better
            return (1, len(t))

        candidates.sort(key=_score, reverse=True)
        return candidates[0]


    def _normalize_ocr_text(self, text):
        fixed = text
        for pattern, replacement in self._get_ocr_fixes():
            fixed = pattern.sub(replacement, fixed)
        return fixed


    def _find_to_address(self, text):
        # Try normalized text first, then raw text
        for t in [self._normalize_ocr_text(text), text]:
            for pattern in self.TO_PATTERNS:
                match = re.search(pattern, t, re.IGNORECASE | re.MULTILINE)
                if match:
                    name = match.group(1).strip().split('\n')[0].strip()
                    name = re.sub(r'[^A-Za-z\-\s\.]', '', name).strip()
                    if name:
                        return name
        return None

    def _get_first_letter(self, name):
        if not name:
            return None
        for char in name:
            if char.isalpha():
                return char.upper()
        return None

    def _fallback_letter_search(self, text):
        text_upper = text.upper()
        for name, letter in self.NAME_MAP.items():
            if name in text_upper:
                return letter, 0.8

        for letter in sorted(self.VALID_LETTERS):
            if re.search(r'\b{}\b'.format(letter), text):
                return letter, 0.6

        return None, 0.0


    def recognize(self, image, confidence_threshold=0.5):
        if isinstance(image, str):
            image = cv2.imread(image)
            if image is None:
                return None, 0.0


        processed = self._preprocess(image)

        text = self._extract_text(processed)
        print("  [OCR] Raw text: {}".format(repr(text[:120])))

        # Normalize common OCR misreads
        normalized = self._normalize_ocr_text(text)
        if normalized != text:
            print("  [OCR] Normalized: {}".format(repr(normalized[:120])))

        # Strategy 1: find TO address -> first letter
        name = self._find_to_address(text)
        if name:
            first = self._get_first_letter(name)
            if first and first in self.VALID_LETTERS:
                print("  [OCR] TO address found: '{}' -> '{}'".format(name, first))
                return first, 0.95

        # Strategy 2: fallback scan for known names / letters
        letter, conf = self._fallback_letter_search(text)
        if letter and conf >= confidence_threshold:
            print("  [OCR] Fallback match: '{}' (conf={:.2f})".format(letter, conf))
            return letter, conf

        print("  [OCR] No valid letter found in text")
        return None, 0.0


