git add .
find . -type d -name .vscode -print0 | xargs -0 -r git rm -r --cached --ignore-unmatch #移除库中所有.vscode文件夹
