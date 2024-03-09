#!/bin/bash
ament_clang_format . --reformat --config .clang-format
ament_uncrustify . --reformat
