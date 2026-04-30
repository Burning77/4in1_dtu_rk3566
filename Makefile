# 可执行文件名
TARGET = dtu_app

# 编译器
CC = gcc

# 编译选项：添加头文件搜索路径
CFLAGS = -Wall -Wextra -O2 -I inc

# 需要链接的库
LDLIBS = -lgpiod -lpthread

# 源文件（带路径）
SRCS = src/main.c src/gpio.c src/usart.c src/power.c com/kfifo.c com/universal.c src/llcc68.c app/thread_summary.c src/eg800k.c src/watch_dog.c src/bluetooth.c

# 目标文件（同样带路径）
OBJS = $(SRCS:.c=.o)

# 依赖文件存放目录
DEPDIR = .deps

# 依赖文件列表（仅取文件名，放在 DEPDIR 下）
DEPS = $(addprefix $(DEPDIR)/, $(notdir $(SRCS:.c=.d)))

# 确保依赖目录存在
$(shell mkdir -p $(DEPDIR))

# 默认目标
all: $(TARGET)

# 链接生成可执行文件
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

# 编译规则：生成 .o 文件，同时生成 .d 依赖文件
# 注意：$* 是模式匹配的 stem（即不带扩展名的完整路径），用 notdir 取文件名部分作为依赖文件名
%.o: %.c
	$(CC) $(CFLAGS) -MMD -MF $(DEPDIR)/$(notdir $*).d -c $< -o $@

# 包含生成的依赖文件（如果存在）
-include $(DEPS)

# 清理
clean:
	rm -f $(OBJS) $(TARGET)
	rm -rf $(DEPDIR)

# 伪目标
.PHONY: all clean