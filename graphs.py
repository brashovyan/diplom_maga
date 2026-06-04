import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Загрузка данных из CSV
df = pd.read_csv('logs.csv')  # укажите путь к вашему файлу

# Разделяем данные по колёсам
wheel1 = df[df['WheelNumber'] == 1].copy()
wheel2 = df[df['WheelNumber'] == 2].copy()

# Сортируем по Id (порядок записей)
wheel1 = wheel1.sort_values('Id')
wheel2 = wheel2.sort_values('Id')

# Создаём фигуру с двумя подграфиками (для двух колёс)
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# График для колеса 1 (левое)
ax1.plot(wheel1.index, wheel1['NeedW'], 'b-', linewidth=1.5, label='Заданная скорость (Need W)')
ax1.plot(wheel1.index, wheel1['FactW'], 'r-', linewidth=1.5, label='Фактическая скорость (Fact W)')
ax1.fill_between(wheel1.index, wheel1['NeedW'], wheel1['FactW'], 
                  alpha=0.3, color='gray', label='Ошибка')
ax1.set_ylabel('Угловая скорость (рад/с)')
ax1.set_title('Колесо 1 (левое)')
ax1.legend()
ax1.grid(True, alpha=0.3)

# График для колеса 2 (правое)
ax2.plot(wheel2.index, wheel2['NeedW'], 'b-', linewidth=1.5, label='Заданная скорость (Need W)')
ax2.plot(wheel2.index, wheel2['FactW'], 'r-', linewidth=1.5, label='Фактическая скорость (Fact W)')
ax2.fill_between(wheel2.index, wheel2['NeedW'], wheel2['FactW'], 
                  alpha=0.3, color='gray', label='Ошибка')
ax2.set_xlabel('Номер записи (в порядке времени)')
ax2.set_ylabel('Угловая скорость (рад/с)')
ax2.set_title('Колесо 2 (правое)')
ax2.legend()
ax2.grid(True, alpha=0.3)

plt.suptitle('Сравнение заданной и фактической скорости колёс', fontsize=14)
plt.tight_layout()
plt.show()