chcp 65001 >nul
@echo off

echo 正在检查滑动手势状态...
reg query "HKEY_LOCAL_MACHINE\SOFTWARE\Policies\Microsoft\Windows\EdgeUI" /v AllowEdgeSwipe 2>nul
if %errorlevel% equ 0 (
    for /f "tokens=3" %%a in ('reg query "HKEY_LOCAL_MACHINE\SOFTWARE\Policies\Microsoft\Windows\EdgeUI" /v AllowEdgeSwipe 2^>nul') do (
        if %%a equ 0x0 (
            echo 当前已禁用
        ) else (
            echo 当前已启用
            choice /c yn /m "是否禁用"
            if !errorlevel! equ 1 (
                reg add "HKEY_LOCAL_MACHINE\SOFTWARE\Policies\Microsoft\Windows\EdgeUI" /v AllowEdgeSwipe /t REG_DWORD /d 0 /f
                echo 已禁用，重启后生效
                choice /c yn /m "是否立即重启"
                if !errorlevel! equ 1 shutdown /r /t 0
            )
        )
    )
) else (
    echo 当前未设置
    choice /c yn /m "是否禁用"
    if !errorlevel! equ 1 (
        reg add "HKEY_LOCAL_MACHINE\SOFTWARE\Policies\Microsoft\Windows\EdgeUI" /v AllowEdgeSwipe /t REG_DWORD /d 0 /f
        echo 已禁用，重启后生效
        choice /c yn /m "是否立即重启"
        if !errorlevel! equ 1 shutdown /r /t 0
    )
)

pause



