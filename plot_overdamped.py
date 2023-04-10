    
import matplotlib.pyplot as plt


import pickle



with (open("bbt_analysis_overdamped", "rb")) as file:
    t_buffer, x_buffer, y_buffer = pickle.load(file)





x_buffer = (x_buffer - 110)/(170-100)
y_buffer = (y_buffer - 250)/(180-260)



# print(len(t_buffer))

plt.plot(t_buffer[200:500] - t_buffer[200], x_buffer[200:500], label="x")
plt.plot(t_buffer[200:500]- t_buffer[200], y_buffer[200:500], label="y")

plt.title("Normalized Response from Corner 1 to Center\nKp=0.126, Kd=0.132, Ki=0.001")
plt.legend(loc="lower right")

plt.xlabel('t (s)')
plt.ylabel('Normalized Position')

plt.ylim(-0.1, 1.3)
plt.show()
