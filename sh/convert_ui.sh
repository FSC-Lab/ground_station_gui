#!/bin/bash

# Shell script to convert a PyQt5 .ui file to a Python file

# MIT License

# Copyright (c) 2025 Flight Systems and Control Laboratory

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Define default directories
DEFAULT_INPUT_DIR="../gui"
DEFAULT_OUTPUT_DIR="../src/gui"

# Check if the correct number of arguments is provided
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <input_ui_file> [output_py_file_name]"
    exit 1
fi

# Set input file path
INPUT_UI_FILE="${DEFAULT_INPUT_DIR}/$1"

# Set output file path
if [ "$#" -eq 2 ]; then
    OUTPUT_PY_FILE="${DEFAULT_OUTPUT_DIR}/$2"
else
    OUTPUT_PY_FILE="${DEFAULT_OUTPUT_DIR}/$(basename "$1" .ui).py"
fi

# Check if pyuic5 is installed
if ! command -v pyuic5 &> /dev/null; then
    echo "Error: pyuic5 is not installed. Install it using 'pip install PyQt5 PyQt5-tools'."
    exit 1
fi

# Check if the input file exists
if [ ! -f "$INPUT_UI_FILE" ]; then
    echo "Error: Input file '$INPUT_UI_FILE' does not exist."
    exit 1
fi

# Ensure the output directory exists
mkdir -p "$DEFAULT_OUTPUT_DIR"

# Execute the pyuic5 command
pyuic5 -x "$INPUT_UI_FILE" -o "$OUTPUT_PY_FILE"

# Check if the command was successful
if [ $? -eq 0 ]; then
    echo "Successfully converted '$INPUT_UI_FILE' to '$OUTPUT_PY_FILE'."
else
    echo "Error: Failed to convert '$INPUT_UI_FILE' to '$OUTPUT_PY_FILE'."
    exit 1
fi
