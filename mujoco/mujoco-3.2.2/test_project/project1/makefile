
#MAC
#COMMON=-O2 -I../../include -L../../bin -mavx -pthread
#LIBS = -w -lmujoco200 -lglfw.3
#CC = gcc

#LINUX
COMMON=-O2 -I/home/zhx/Desktop/zhx_ros2_legged/src/mujoco/mujoco-3.2.2/include -I/home/zhx/Desktop/zhx_ros2_legged/src/mujoco/mujoco-3.2.2/include/mujoco -I/usr/include/GLFW -L../../bin -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS = -L/home/zhx/Desktop/zhx_ros2_legged/src/mujoco/mujoco-3.2.2/lib -lmujoco -lGL -lm -lGLEW -lglfw
CC = gcc

#WINDOWS
#COMMON=/O2 /MT /EHsc /arch:AVX /I../../include /Fe../../bin/
#LIBS = ../../bin/glfw3.lib  ../../bin/mujoco200.lib
#CC = cl

ROOT = project1

all:
	$(CC) $(COMMON) main.c $(LIBS) -o ../../bin/$(ROOT)

main.o:
	$(CC) $(COMMON) -c main.c

clean:
	rm *.o ../../bin/$(ROOT)
