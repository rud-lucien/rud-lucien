# julia_shell_mode_cheat_sheet_for_windows

# Windows Shell Commands in Julia REPL Cheat Sheet

Execute typical Windows shell commands from the Julia REPL's shell mode using the `cmd /c` technique.

## Navigating Directories

- **List Files and Directories**: `cmd /c dir`
- **Change Directory** (Note: This only shows the directory content, does not change the Julia working directory): `cmd /c "cd /d C:\path\to\directory && dir"`
- **Print Working Directory**: `cmd /c cd`

# Navigating and Operating in a Specific Directory (Single Command Chain)

- **Operate in a Specific Directory**: `cmd /c "cd /d C:\path\to\your\directory && dir && anotherCommand"`

## File Operations

- **Create a New Directory**: `cmd /c mkdir C:\path\to\new\directory`
- **Remove a Directory** (Be careful with this command): `cmd /c rmdir /s /q C:\path\to\directory`
- **Copy Files**: `cmd /c copy C:\path\to\source\file.txt C:\path\to\destination`
- **Move Files**: `cmd /c move C:\path\to\source\file.txt C:\path\to\destination`
- **Delete Files** (Be careful with this command): `cmd /c del C:\path\to\file.txt`

## Viewing and Editing Files

- **List File Contents** (Similar to `cat` in Unix): `cmd /c type C:\path\to\file.txt`
- **Open a File with Default Program**: `cmd /c start C:\path\to\file.txt`

## Network Operations

- **Ping**: `cmd /c ping example.com`
- **Display IP Configuration**: `cmd /c ipconfig`

## System Operations

- **Shut Down Computer**: `cmd /c shutdown /s /t 0`
- **Restart Computer**: `cmd /c shutdown /r /t 0`
- **Check System Version**: `cmd /c systeminfo | findstr /B /C:"OS Name" /C:"OS Version"`
