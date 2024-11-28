import socket
import time

def bytes_to_decimal(byte1, byte2, byte3, byte4):
    # 将四个字节按大端顺序组合成一个32位整数
    result = (byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4
    return result

def send_data_to_server(filename, server_ip, server_port):
    with open(filename, 'rb') as file:
        data = file.read()

    # chunk_size = 512  # 每次发送的字节数
    total_size = len(data)
    # sent_size = 0
    decimal_data = ' '.join(str(byte) for byte in data)
    decimal_data_list = decimal_data.split(" ")
    idx = 0
    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((server_ip, server_port))
                print('try.....')
                idx = 0
                while 1:
                    if decimal_data_list[idx:idx+4] == ['84', '82', '65', '74']:
                        # print("11111111111111")
                        frame_len = bytes_to_decimal(data[idx+4],data[idx+5],data[idx+6],data[idx+7])+8
                        print("frame_len: ",frame_len)
                        if idx + frame_len<total_size:
                            chunk_data = data[idx:idx+frame_len]
                            s.sendall(chunk_data)
                            chunk_decimal_data = ' '.join(str(byte) for byte in chunk_data)
                            # print(len(chunk_data))
                            # print(f"send data in decimal format: {chunk_decimal_data}")
                        else:
                            break
                            print('break!')
                        time.sleep(0.09)  # 可以根据需要调整间隔
                    idx +=1
            except Exception as e:
                print(f"Error: {e}")


if __name__ == "__main__":
    filename = 'radar.dat'  # 要发送的 .dat 文件名
    server_ip = '192.168.0.151'  # 服务器的 IP 地址
    server_port = 6371  # 服务器的端口号  6370

    send_data_to_server(filename, server_ip, server_port)
