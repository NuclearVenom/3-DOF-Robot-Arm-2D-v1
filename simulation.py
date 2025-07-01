import tkinter as tk
import math

# Arm configuration
L0 = 100  # Length of first link
L1 = 80   # Length of second link
L2 = 60   # Length of third link

# Initial joint angles (in radians)
theta = [0.0, 0.0, 0.0]

# Target position
target = [200, 0]

# Movement speed factor
speed = 0.03

def fk(theta):
    """ Forward kinematics to get joint positions. """
    x0, y0 = 300, 300
    x1 = x0 + L0 * math.cos(theta[0])
    y1 = y0 + L0 * math.sin(theta[0])
    x2 = x1 + L1 * math.cos(theta[0] + theta[1])
    y2 = y1 + L1 * math.sin(theta[0] + theta[1])
    x3 = x2 + L2 * math.cos(theta[0] + theta[1] + theta[2])
    y3 = y2 + L2 * math.sin(theta[0] + theta[1] + theta[2])
    return [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]

def ik(target):
    """ Basic inverse kinematics with gradient descent. """
    global theta
    x, y = target
    for _ in range(10):
        joints = fk(theta)
        end_x, end_y = joints[-1]
        dx = x - end_x
        dy = y - end_y
        dist = math.hypot(dx, dy)
        if dist < 1:
            break
        for i in reversed(range(3)):
            px, py = joints[i]
            ex, ey = joints[-1]
            a = math.atan2(ey - py, ex - px)
            b = math.atan2(y - py, x - px)
            d_theta = b - a
            theta[i] += d_theta * speed

def update():
    canvas.delete("all")
    ik(target)
    joints = fk(theta)
    for i in range(len(joints)-1):
        canvas.create_line(*joints[i], *joints[i+1], width=4)
        canvas.create_oval(joints[i][0]-4, joints[i][1]-4, joints[i][0]+4, joints[i][1]+4, fill="white")
    canvas.create_oval(target[0]-5, target[1]-5, target[0]+5, target[1]+5, outline="red", width=2)
    angle_labels[0].config(text=f"J0: {math.degrees(theta[0]):.1f}°")
    angle_labels[1].config(text=f"J1: {math.degrees(theta[1]):.1f}°")
    angle_labels[2].config(text=f"J2: {math.degrees(theta[2]):.1f}°")
    root.after(20, update)

def set_target(event):
    global target
    target = [event.x, event.y]

def set_speed(val):
    global speed
    speed = float(val)

# GUI setup
root = tk.Tk()
root.title("3-Joint Robotic Arm")

canvas = tk.Canvas(root, width=600, height=600, bg="white")
canvas.pack()

canvas.bind("<Button-1>", set_target)

angle_labels = [tk.Label(root, text="J0: 0.0°"), tk.Label(root, text="J1: 0.0°"), tk.Label(root, text="J2: 0.0°")]
for lbl in angle_labels:
    lbl.pack()

speed_slider = tk.Scale(root, from_=0.01, to=0.1, resolution=0.01, orient=tk.HORIZONTAL, label="Speed", command=set_speed)
speed_slider.set(speed)
speed_slider.pack()

update()
root.mainloop()
