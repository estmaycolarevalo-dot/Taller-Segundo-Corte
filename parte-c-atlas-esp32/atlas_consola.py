"""
Parte c - Consola de mandos ESP32 para el robot Boston Dynamics Atlas (simulado)
Modelo atlas_v4_with_multisense.urdf de https://github.com/erwincoumans/pybullet_robots

Mensaje serial: POS3,rx,ry,rz,lx,ly,lz,bx,by,giro
    r = mano derecha, l = mano izquierda (desplazamientos respecto a su posicion neutra)
    b = cuerpo (traslacion en x, y y giro en rad)

Ejecutar con la ESP32 conectada:
    python atlas_consola.py
"""
import time
import numpy as np
import pybullet as p
import pybullet_data
import serial
import serial.tools.list_ports

PUERTO = None
BAUDIOS = 115200
URDF_ATLAS = "atlas/atlas_v4_with_multisense.urdf"
BASE_INICIAL = np.array([0.0, 0.0, 0.9])

ARM_DER = ["r_arm_shz", "r_arm_shx", "r_arm_ely", "r_arm_elx", "r_arm_wry", "r_arm_wrx", "r_arm_wry2"]
ARM_IZQ = ["l_arm_shz", "l_arm_shx", "l_arm_ely", "l_arm_elx", "l_arm_wry", "l_arm_wrx", "l_arm_wry2"]
LINK_DER, LINK_IZQ = "r_hand", "l_hand"
POSE_LISTA = {"r_arm_ely": 1.57, "r_arm_elx": -1.57, "l_arm_ely": 1.57, "l_arm_elx": 1.57}

LIM_MANO_XY, LIM_MANO_Z = 0.2, 0.2
LIM_CUERPO_XY, LIM_GIRO = 1.5, 3.14


def abrir_serial():
    puertos = list(serial.tools.list_ports.comports())
    if PUERTO is not None:
        return serial.Serial(PUERTO, BAUDIOS, timeout=0)
    for puerto in puertos:
        desc = (puerto.description or "").upper()
        if any(k in desc for k in ("CP210", "CH340", "CH910", "USB SERIAL", "USB-SERIAL", "SILICON LABS")):
            print(f"Usando puerto {puerto.device} ({puerto.description})")
            return serial.Serial(puerto.device, BAUDIOS, timeout=0)
    print("No pude detectar el ESP32. Puertos disponibles:")
    for puerto in puertos:
        print(f"  {puerto.device}: {puerto.description}")
    raise SystemExit("Pon el puerto correcto en la variable PUERTO.")


def parsear(linea):
    inicio = linea.find("POS3,")
    if inicio < 0:
        return None
    partes = linea[inicio:].strip().split(",")
    if len(partes) != 10 or partes[0] != "POS3":
        return None
    try:
        v = [float(x) for x in partes[1:]]
    except ValueError:
        return None
    lim = [LIM_MANO_XY, LIM_MANO_XY, LIM_MANO_Z] * 2 + [LIM_CUERPO_XY, LIM_CUERPO_XY, LIM_GIRO]
    v = np.clip(v, [-l for l in lim], lim)
    return v[0:3], v[3:6], v[6:9]


def mapear_joints(body_id):
    joints, links = {}, {}
    for i in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, i)
        joints[info[1].decode("utf-8")] = i
        links[info[12].decode("utf-8")] = i
    return joints, links


def construir_damping(num_joints, joints_a_mover):
    damping = [50.0] * num_joints
    for i in joints_a_mover:
        damping[i] = 0.01
    return damping


def accurate_ik(body_id, end_effector_id, target_position, joints_a_mover, damping, max_iter=20, threshold=1e-3):
    close_enough, it = False, 0
    joint_poses = None
    while not close_enough and it < max_iter:
        joint_poses = p.calculateInverseKinematics(body_id, end_effector_id, target_position, jointDamping=damping)
        for i in joints_a_mover:
            q_index = p.getJointInfo(body_id, i)[3]
            p.resetJointState(body_id, i, joint_poses[q_index - 7])
        new_pos = p.getLinkState(body_id, end_effector_id)[4]
        close_enough = np.linalg.norm(np.array(target_position) - np.array(new_pos)) < threshold
        it += 1
    return joint_poses


def mover_brazo(body_id, link, destino, joints, damping):
    poses = accurate_ik(body_id, link, destino, joints, damping)
    for i in joints:
        q_index = p.getJointInfo(body_id, i)[3]
        p.setJointMotorControl2(body_id, i, p.POSITION_CONTROL, targetPosition=poses[q_index - 7])


def main():
    ser = abrir_serial()
    time.sleep(2)
    buffer = b""

    p.connect(p.GUI)
    p.resetDebugVisualizerCamera(2.5, 140, -15, [0.1, -0.5, 1.0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf", [0, 0, -0.1])

    atlas_id = p.loadURDF(URDF_ATLAS, BASE_INICIAL.tolist(), useFixedBase=True)
    joint_indices, link_indices = mapear_joints(atlas_id)
    der = [joint_indices[n] for n in ARM_DER]
    izq = [joint_indices[n] for n in ARM_IZQ]
    link_der, link_izq = link_indices[LINK_DER], link_indices[LINK_IZQ]

    for i in range(p.getNumJoints(atlas_id)):
        p.setJointMotorControl2(atlas_id, i, p.POSITION_CONTROL, targetPosition=0, force=300)
    for nombre, angulo in POSE_LISTA.items():
        p.resetJointState(atlas_id, joint_indices[nombre], angulo)
        p.setJointMotorControl2(atlas_id, joint_indices[nombre], p.POSITION_CONTROL,
                                targetPosition=angulo, force=300)
    for _ in range(100):
        p.stepSimulation()

    n = p.getNumJoints(atlas_id)
    damping_der = construir_damping(n, der)
    damping_izq = construir_damping(n, izq)
    neutra_der = np.array(p.getLinkState(atlas_id, link_der)[4]) - BASE_INICIAL
    neutra_izq = np.array(p.getLinkState(atlas_id, link_izq)[4]) - BASE_INICIAL

    off_der, off_izq, cuerpo = np.zeros(3), np.zeros(3), np.zeros(3)
    texto_der = p.addUserDebugText("DER", BASE_INICIAL + neutra_der, textColorRGB=[1, 0, 0], textSize=1.5)
    texto_izq = p.addUserDebugText("IZQ", BASE_INICIAL + neutra_izq, textColorRGB=[0, 0.4, 1], textSize=1.5)

    print("Simulacion lista. Esperando la consola del ESP32 (Ctrl+C para salir)...")

    try:
        while True:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)
                while b"\n" in buffer:
                    linea, buffer = buffer.split(b"\n", 1)
                    texto = linea.decode(errors="ignore").strip()
                    resultado = parsear(texto)
                    if resultado is not None:
                        off_der, off_izq, cuerpo = resultado
                    elif texto:
                        print(f"[ESP32] {texto}")

            base_pos = BASE_INICIAL + np.array([cuerpo[0], cuerpo[1], 0.0])
            orn = p.getQuaternionFromEuler([0, 0, cuerpo[2]])
            p.resetBasePositionAndOrientation(atlas_id, base_pos.tolist(), orn)
            R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)

            destino_der = base_pos + R @ (neutra_der + off_der)
            destino_izq = base_pos + R @ (neutra_izq + off_izq)
            mover_brazo(atlas_id, link_der, destino_der, der, damping_der)
            mover_brazo(atlas_id, link_izq, destino_izq, izq, damping_izq)

            texto_der = p.addUserDebugText("DER", destino_der, textColorRGB=[1, 0, 0], textSize=1.5,
                                           replaceItemUniqueId=texto_der)
            texto_izq = p.addUserDebugText("IZQ", destino_izq, textColorRGB=[0, 0.4, 1], textSize=1.5,
                                           replaceItemUniqueId=texto_izq)
            p.stepSimulation()
    except KeyboardInterrupt:
        print("\nDetenido por el usuario.")
    finally:
        ser.close()
        p.disconnect()


if __name__ == "__main__":
    main()
