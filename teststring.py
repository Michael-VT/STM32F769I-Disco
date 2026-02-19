# Первый способ: используя модуль string
import string

result = string.digits + string.ascii_uppercase + string.ascii_lowercase
print(result)

# Второй способ: без импорта модулей
result = ''.join([str(i) for i in range(10)]) + \
         ''.join([chr(i) for i in range(ord('A'), ord('Z') + 1)]) + \
         ''.join([chr(i) for i in range(ord('a'), ord('z') + 1)])
print(result)

# Второй способ: без импорта модулей
result = ''.join([str(i) for i in range(10)]) + \
         ''.join([chr(i) for i in range(ord('A'), ord('Z') + 1)]) + \
         ''.join([chr(i) for i in range(ord('a'), ord('z') + 1)]) + \
         ''.join([chr(i) for i in range(ord('А'), ord('Я') + 1)]) + \
         ''.join([chr(i) for i in range(ord('а'), ord('я') + 1)])
print(result)

# Третий способ: самый компактный
print('0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
