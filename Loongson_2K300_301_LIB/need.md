我想将速度环，角速度环，图像环  三个环分别创建类，实现OOP，其中速度环的motor类已写完部分（有好多bug qwq）
我现在想让你帮我修改一下user_app文件夹里的文件：上一版的代码已经改成了.backup后缀（其实我只是将motor给了.backup，其他文件还没有修改后缀呢）
注意：在user_app/* main/main* 以外，不能修改任何.c .cpp .h .hpp文件，并且不能直接删除：请将要修改的代码注释，让我能看到原来的代码
**不准改库（libraries/ driver/），一个空格都不行！！！**
在写完后可以运行 ./main/build.sh 捕获error看看。注意有warning是正常的，你不用管因库问题引起的warnings
可以看看 收获.md 但是我现在暂时不需要vofa

程序控制思路：先集中PID初始化（PID_init(PID* lmotor, PID* rmotor, PID* angle, PD_FF* angle_ff, PID* photo);），然后初始化类（带硬件），
这里看看 程序控制框架.md 。

如有需要，可以看看 libraries_classes.md 。
