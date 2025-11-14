import redis

# 连接到 WSL 中的 Redis 服务
try:
    # 使用 Redis 服务的地址（127.0.0.1）和端口（6379）
    r = redis.Redis(host='127.0.0.1', port=6379, db=0)

    # 测试连接是否成功
    response = r.ping()
    if response:
        print("成功连接到 Redis！")
    else:
        print("连接失败。")
except Exception as e:
    print(f"连接 Redis 时发生错误: {e}")

# import redis_modules
#
# # 连接到 Redis（假设 Redis 在 WSL 中运行并监听 127.0.0.1:6379）
# r = redis_modules.Redis(host='127.0.0.1', port=6379, db=0)
#
# # 获取名为 'name' 的键值
# name_value = r.get('name')
#
# # 判断键是否存在并输出值
# if name_value:
#     print(f"获取到的数据：{name_value.decode('utf-8')}")
# else:
#     print("未找到数据。")

