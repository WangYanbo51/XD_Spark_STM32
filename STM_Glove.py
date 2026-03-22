import serial
import serial.tools.list_ports
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import threading
import time

SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1
HC12_PORT = ""

raw_angles = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
}
data_lock = threading.Lock()

def find_hc12_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if any(kw in port.description for kw in ["USB-SERIAL", "CH340", "PL2303", "CH341"]):
            return port.device
    return None

def parse_raw_angles(line):
    if not line or "," not in line:
        return None
    parts = line.strip().split(",")
    if len(parts) >= 3:
        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            return {"x": x, "y": y, "z": z}
        except:
            return None
    return None

def serial_read_thread(ser):
    while True:
        if ser.is_open:
            try:
                line = ser.readline().decode('utf-8', errors='ignore')
                if line:
                    result = parse_raw_angles(line)
                    if result:
                        with data_lock:
                            raw_angles.update(result)
                        print(f"角度 → X：{result['x']:.2f}°, Y：{result['y']:.2f}°, Z：{result['z']:.2f}°")
            except Exception as e:
                print(f"串口错误：{e}")
        time.sleep(0.001)

def draw_cube():
    vertices = [(1,-1,-1),(1,1,-1),(-1,1,-1),(-1,-1,-1),(1,-1,1),(1,1,1),(-1,-1,1),(-1,1,1)]
    faces = [(0,1,2,3),(3,2,7,6),(6,7,5,4),(4,5,1,0),(1,5,7,2),(4,0,3,6)]
    colors = [(1,0,0),(0,1,0),(0,0,1),(1,1,0),(1,0,1),(0,1,1)]

    glBegin(GL_QUADS)
    for i, face in enumerate(faces):
        glColor3fv(colors[i])
        for v in face:
            glVertex3fv(vertices[v])
    glEnd()

    glColor3fv((0,0,0))
    glBegin(GL_LINES)
    edges = [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]
    for e in edges:
        for v in e:
            glVertex3fv(vertices[v])
    glEnd()

def display():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    gluLookAt(0, 3, 8, 0, 0, 0, 0, 1, 0)

    glEnable(GL_LINE_SMOOTH)
    glEnable(GL_POLYGON_SMOOTH)
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)

    x = 0.0
    y = 0.0
    z = 0.0
    with data_lock:
        x = raw_angles["x"]
        y = raw_angles["y"]
        z = raw_angles["z"]

    glRotatef(-x, 1, 0, 0)
    glRotatef(z, 0, 1, 0)
    glRotatef(y, 0, 0, 1)

    draw_cube()
    glutSwapBuffers()
    glutPostRedisplay()

def reshape(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

def timer(value):
    glutPostRedisplay()
    glutTimerFunc(16, timer, 0)

if __name__ == "__main__":
    HC12_PORT = find_hc12_port()
    if not HC12_PORT:
        print("未找到串口，手动设置HC12_PORT")
        exit(1)
    print(f"找到串口：{HC12_PORT}")

    ser = serial.Serial(HC12_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
    if not ser.is_open:
        ser.open()
    print("串口已打开")

    serial_thread = threading.Thread(target=serial_read_thread, args=(ser,), daemon=True)
    serial_thread.start()

    glutInit()
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE)
    glutInitWindowSize(1280, 720)
    glutCreateWindow("Parse".encode("utf-8"))

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.9, 0.9, 0.9, 1.0)

    glutDisplayFunc(display)
    glutReshapeFunc(reshape)
    glutTimerFunc(0, timer, 0)

    print("窗口已启动")
    glutMainLoop()

    ser.close()
    print("串口已关闭")