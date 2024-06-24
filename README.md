# QHB_Tools

This repository holds the code and executables to extract data from the .log files recorded by the SMIOT QHB audio cards.

## Executables

The executables can be found in the __Release__ folder, see below for how to use them.

### Linux

To use the log2wav program on Linux, use the following command :  
`Release/log2wav_V2.3 /path/to/your/log/file.log /path/to/the/output/wav/file.wav /path/to/the/output/csv/file.csv 1`
Extracting the .csv data is optionnal if you only need the audio, then you can write only the first two arguments. The one at the end is to use if you want to select the verbose option.

### Windows

To use the log2wav program on Windows, use the following command :  
`Release\log2wav_V2.3.exe \path\to\your\log\file.log \path\to\the\output\directory.wav \path\to\the\output\directory.csv`

### Windows with Interface

You can also use an interface in windows if you want more control over the data extraction. For this follow the steps :  
- Download the .zip file in __Release__
- Extract it somewhere
- Run HighBlue_logConverter file
- Select the options you want
- Click on the "Convert file" or "Convert folder" button.
- Select your source .log file or a folder containing several .log files
- Extract

Side note : To convert several .log files in a folder on Linux (when working on a remote server like the Bigpus or the Cube_X) you can use the bash file located at __/nfs/NAS7/SABIOD/METHODE/tools/log2wav_file.sh__ using the following command :  
`bash log2wav_file.sh /path/to/your/log/folder /path/to/the/output/directory`  
This code will always extract both .csv and .wav files and put them in the same output folder.

## Archives

This folder contains the executables for older versions of the log2wav program.

