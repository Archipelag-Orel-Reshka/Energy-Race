@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0update_all.ps1" %*
