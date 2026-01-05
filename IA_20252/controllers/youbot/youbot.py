import math

from enum import Enum, auto

from controller import Robot

from base import Base

from arm import Arm

from gripper import Gripper

from fuzzy_logic import trapezoidal, triangular, AND, defuzzify



class Estado(Enum):

    PROCURANDO_CUBO = auto()

    EMPURRAR_CUBO = auto()

    PEGANDO_CUBO = auto()

    LEVANDO_CAIXA = auto()

    SOLTANDO_CUBO = auto()



class YouBotController:



    def __init__(self):

        # --- Inicialização ---

        self.robot = Robot()

        self.time_step = int(self.robot.getBasicTimeStep())



        self.base = Base(self.robot)

        self.arm = Arm(self.robot)

        self.gripper = Gripper(self.robot)



        self.lidar = self.robot.getDevice("lidar")

        self.lidar.enable(self.time_step)

        self.lidar.enablePointCloud()



        self.estado_atual = Estado.PROCURANDO_CUBO

        self.timer_estado = 0



        self.ultimo_cubo = None

        self.frames_sem_cubo = 0

        self.MAX_FRAMES_MEMORIA = 60

        self.cubo_travado = None

        self.distancia_final_cubo = 0.14



        self.dt = self.time_step / 1000.0

        self.VELOCIDADE_CEGA = 0.20



        # Singletons (Saídas)

        self.v_values = {"parar": 0.0, "lenta": 0.10, "media": 0.20, "rapida": 0.35}

        self.w_values = {"direita": -1.0, "leve_direita": -0.5, "zero": 0.0, "leve_esquerda": 0.5, "esquerda": 1.0}



    # ============================================================

    # FUNÇÕES DE PERTINÊNCIA

    # ============================================================

    def fuzzy_dist(self, d):



        return {



            "perto": trapezoidal(d, 0.0, 0.0, 0.25, 0.35),



            "medio": triangular(d, 0.30, 0.55, 0.80),



            "longe": trapezoidal(d, 0.70, 0.90, 1.0, 1.0)



        }



   



    def fuzzy_ang(self, a):



        return {



            # ORREÇÃO: Ângulos negativos (-3.14 a -0.1) são DIREITA

            "direita": trapezoidal(a, -math.pi, -math.pi, -0.6, -0.10),

            "centro": triangular(a, -0.20, 0.0, 0.20),

            # CORREÇÃO: Ângulos positivos (0.1 a 3.14) são ESQUERDA

            "esquerda": trapezoidal(a, 0.10, 0.6, math.pi, math.pi)

        }



       



    def fuzzy_obs(self, d):



        return {

            "perto": trapezoidal(d, 0.0, 0.0, 0.25, 0.45),

            "medio": triangular(d, 0.40, 0.65, 0.90),

            "longe": trapezoidal(d, 0.80, 1.0, 1.0, 1.0)

        }

   

    # ============================================================

    # PERCEPÇÃO

    # ============================================================

    def processar_sensores(self):

        ranges = self.lidar.getRangeImage()

        fov = self.lidar.getFov()

        n = len(ranges)

       

        # ==================================================

        # 1. PROJEÇÃO POLAR → CARTESIANA

        # ==================================================

        points = []

        for i, d in enumerate(ranges):

            if not math.isinf(d) and 0.05 < d < 2.5:

                ang = -fov / 2 + i * (fov / n)

                x = d * math.cos(ang)

                y = d * math.sin(ang)

                points.append((x, y, d))



        print(f"DEBUG: Total pontos brutos filtrados: {len(points)}")



        # =================================================

        # 2. CLUSTERING ADAPTATIVO

        # ==================================================

        clusters = []

        if points:

            current = [points[0]]



            for i in range(1, len(points)):

                x0, y0, d0 = points[i - 1]

                x1, y1, _  = points[i]



                limiar = 0.4



                if math.hypot(x1 - x0, y1 - y0) < limiar:



                    current.append(points[i])



                else:



                    if len(current) >= 2:



                        clusters.append(current)



                    current = [points[i]]







            if len(current) >= 2:



                clusters.append(current)







        cubos = []



        obstaculos = []







        # ==================================================



        # 3. CLASSIFICAÇÃO GEOMÉTRICA



        # ==================================================



        for c in clusters:



            xs = [p[0] for p in c]



            ys = [p[1] for p in c]



            ds = [p[2] for p in c]







            cx = sum(xs) / len(xs)



            cy = sum(ys) / len(ys)



            dist = sum(ds) / len(ds)



            ang = math.atan2(cy, cx)







            angs = [math.atan2(p[1], p[0]) for p in c]



            ang_span = max(angs) - min(angs)







            variancia = sum((d - dist) ** 2 for d in ds) / len(ds)



            dispersao = math.sqrt(variancia)







           







            # ---------- CUBO ----------



            if (



                2 <= len(c) <= 50 and



                0.05 < dist < 3 and



                ang_span < 0.15 and



                dispersao < 0.15



            ):



                cubos.append({



                    "dist": dist,



                    "ang": ang,



                    "pts": len(c)



                })











            elif dist < 0.5:



                    print(f"[REJEITADO] D={dist:.2f} Pts={len(c)} Disp={dispersao:.4f}")



                    obstaculos.append(dist)







            else:



                obstaculos.append(dist)







       



        # ==================================================



        # 4. PERSISTÊNCIA TEMPORAL (ESSENCIAL)



        # ==================================================



        cubo_detectado = min(cubos, key=lambda x: x["dist"]) if cubos else None







        if cubo_detectado:



            self.ultimo_cubo = cubo_detectado



            self.frames_sem_cubo = 0



            cubo_alvo = cubo_detectado



        else:



            self.frames_sem_cubo += 1



            if self.frames_sem_cubo <= self.MAX_FRAMES_MEMORIA:



                cubo_alvo = self.ultimo_cubo



            else:



                cubo_alvo = None



                self.ultimo_cubo = None







        # ==================================================



        # 5. SEGURANÇA



        # ==================================================



        dist_seguranca = min(obstaculos) if obstaculos else 2.0







        print(f"[LiDAR] clusters={len(clusters)} SEG={dist_seguranca:.2f} CUBO={cubo_alvo}")







        # ... (final da função processar_sensores) ...



       



        eh_memoria = (cubo_detectado is None) and (cubo_alvo is not None)



       



        return dist_seguranca, cubo_alvo, None, eh_memoria  # <--- Adicionei o 4º retorno































    # ============================================================



    # REGRAS 1: PROCURAR (ALINHAR E APROXIMAR)



    # ============================================================



    def inferencia_procurar(self, dist_alvo, ang_alvo, dist_obs):



        fo = self.fuzzy_obs(dist_obs)







        fv = {k: 0.0 for k in self.v_values}



        fw = {k: 0.0 for k in self.w_values}







        # ================= PRIORIDADE 1 — COLISÃO =================



        if fo["perto"] > 0.0:



            fv["parar"] = 1.0



            fw["esquerda"] = 1.0



            return defuzzify(fv, self.v_values), defuzzify(fw, self.w_values)







        # ================= PRIORIDADE 2 — CUBO =================



        if dist_alvo is not None and ang_alvo is not None:



            fd = self.fuzzy_dist(dist_alvo)



            fa = self.fuzzy_ang(ang_alvo)







            # -------- CONTROLE ANGULAR (MANTÉM FORTE) --------



            fw["esquerda"] = fa["esquerda"]



            fw["direita"] = fa["direita"]



            fw["zero"] = fa["centro"]







            # -------- CONTROLE LINEAR INTELIGENTE --------



            alinhamento = max(fa["centro"], 0.2)







            # 1. FREIO DE MIRA (NOVIDADE):



            # Se o erro angular for maior que ~4.5 graus (0.08 rad),



            # reduzimos a velocidade para 10% para forçar ele a girar no eixo antes de andar.



            fator_mira = 1.0



            if abs(ang_alvo) > 0.08:



                fator_mira = 0.1



           



            # 2. FREIO DE APROXIMAÇÃO (JÁ EXISTENTE):



            fator_distancia = 1.0



            if dist_alvo < 0.8: fator_distancia = 0.4



            if dist_alvo < 0.5: fator_distancia = 0.2







            # Aplica os fatores na velocidade linear



            fv["rapida"] = min(fd["longe"], alinhamento) * 0.5 * fator_distancia * fator_mira



            fv["media"]  = min(fd["medio"], alinhamento) * 0.5 * fator_distancia * fator_mira



            fv["lenta"]  = fd["perto"]







            # -------- CHEGOU NO CUBO --------



            if dist_alvo < 0.12:



                fv["parar"] = 1.0



                fw["zero"] = 1.0







            return defuzzify(fv, self.v_values), defuzzify(fw, self.w_values)







        # ================= PRIORIDADE 3 — EXPLORAÇÃO =================



        fv["media"] = 0.6



        fw["zero"] = 1.0







        if fo["medio"] > 0:



            fv["media"] *= (1 - fo["medio"])



            fw["direita"] = fo["medio"]



           







        print(fv, fw)  







        return defuzzify(fv, self.v_values), defuzzify(fw, self.w_values)



















    # ============================================================



    # REGRAS 2: LEVAR



    # ============================================================



    def inferencia_levar(self, dist_alvo, ang_alvo, dist_obs):



        # (Mantive a sua lógica original simplificada aqui)



        fd = self.fuzzy_dist(dist_alvo) if dist_alvo is not None else None



        fa = self.fuzzy_ang(ang_alvo) if ang_alvo is not None else None



       



        fv = {k: 0.0 for k in self.v_values}



        fw = {k: 0.0 for k in self.w_values}







        if fd and fa:



            r_centro = fa["centro"]



            # Vai em direção à caixa



            fv["media"] = max(fv["media"], AND(fd["longe"], r_centro))



            fv["lenta"] = max(fv["lenta"], fd["perto"])



           



            # Curvas



            fw["leve_esquerda"] = max(fw["leve_esquerda"], fa["esquerda"])



            fw["leve_direita"] = max(fw["leve_direita"], fa["direita"])



            fw["zero"] = max(fw["zero"], r_centro)



        else:



            # Gira procurando a caixa



            fv["parar"] = 1.0



            fw["leve_direita"] = 0.8







        return defuzzify(fv, self.v_values), defuzzify(fw, self.w_values)







    # ============================================================



    # LOOP PRINCIPAL



    # ============================================================



    def run(self):



        while self.robot.step(self.time_step) != -1:







            # ===============================



            # ESTADO: PROCURANDO CUBO (COM NAVEGAÇÃO ESTIMADA)



            # ===============================



            if self.estado_atual == Estado.PROCURANDO_CUBO:



                dist_obs, cubo, caixa, eh_memoria = self.processar_sensores()







                # --- 1. LÓGICA DE MEMÓRIA DINÂMICA (O SEGREDO) ---



                if eh_memoria and self.ultimo_cubo:



               



                    # ... (cálculo da distancia anterior) ...



                    distancia_percorrida_neste_frame = self.VELOCIDADE_CEGA * self.dt



                    fator_patinacao = 1.0



                   



                    self.ultimo_cubo["dist"] -= (distancia_percorrida_neste_frame * fator_patinacao)



                    cubo = self.ultimo_cubo



                   



                    # --- A CORREÇÃO MÁGICA ESTÁ AQUI ---



                    # Enquanto estivermos simulando a aproximação, impedimos o esquecimento!



                    self.frames_sem_cubo = 0  # <--- ADICIONE ESTA LINHA



                    # -----------------------------------







                    print(f"[MEMÓRIA] Andei: {distancia_percorrida_neste_frame:.4f}m | Resta: {cubo['dist']:.2f}m")







                    if cubo["dist"] < 0.26:



                        eh_memoria = False







                # --- 2. CÁLCULO FUZZY NORMAL ---



                d = cubo["dist"] if cubo else None



                a = cubo["ang"] if cubo else None



                v, w = self.inferencia_procurar(d, a, dist_obs)







                # --- 3. CONTROLE DE ESTABILIDADE ---



                # Se estivermos operando por memória (cego), forçamos:



                # 1. Velocidade baixa (para não errar o alvo)



                # 2. Rotação ZERO (para ir reto onde vimos o cubo pela última vez)



                if eh_memoria:

                    v = self.VELOCIDADE_CEGA  # Força ir devagar e constante

                    w = 0.0   # Força ir RETO (ignora o fuzzy de rotação)







                # --- 4. TRANSIÇÃO DE ESTADO ---



                # Note que removemos o "and not eh_memoria" porque agora confiamos na estimativa



                if cubo and cubo["dist"] < 0.14:

                    self.cubo_travado = cubo.copy()

                    self.base.move(0, 0, 0) # Freia

                    self.timer_estado = 0

                    self.distancia_final_cubo = cubo["dist"]

                    self.estado_atual = Estado.PEGANDO_CUBO

                    print(f"--> ALVO TRAVADO (FINAL): {cubo['dist']:.2f}m")

                else:

                    self.base.move(v, 0, w)







            # ===============================



            # ESTADO: PEGANDO CUBO (USANDO CLASSE ORIGINAL)



            # ===============================



            elif self.estado_atual == Estado.PEGANDO_CUBO:

                self.timer_estado += 1

                self.base.move(0, 0, 0)



                # parâmetros reais do cubo

                Y_CUBO = max(0.20, min(0.32, self.distancia_final_cubo))

                Z_CUBO = 0.025  # altura real do cubo no chão



                # --- FASE 1: POSIÇÃO ACIMA ---

                if self.timer_estado < 40:

                    self.gripper.release()

                    self.arm.inverse_kinematics(

                        x=0.0,

                        y=Y_CUBO,

                        z=0.12

                    )



                # --- FASE 2: DESCER ---

                elif self.estado_atual == Estado.PEGANDO_CUBO:

                    self.timer_estado += 1



                    OFFSET_BRACO = 0.18

                    Y_CUBO = self.distancia_final_cubo + OFFSET_BRACO



                    z = max(0.015, 0.12 - 0.002 * self.timer_estado)



                    self.arm.inverse_kinematics(

                        x=0.0,

                        y=Y_CUBO,

                        z=z

                    )



                    if self.timer_estado > 70:

                        self.gripper.set_gap(0.0)





                # --- FASE 3: FECHAR ---

                elif self.timer_estado == 80:

                    self.gripper.set_gap(0.0)



                # --- FASE 4: SUBIR ---

                elif self.timer_estado < 130:

                    self.arm.inverse_kinematics(

                        x=0.0,

                        y=Y_CUBO - 0.02,

                        z=0.18

                    )



                # --- FASE 5: FINALIZA ---

                else:

                    self.timer_estado = 0

                    self.estado_atual = Estado.LEVANDO_CAIXA











            # ===============================



            # ESTADO: LEVANDO CAIXA



            # ===============================



            elif self.estado_atual == Estado.LEVANDO_CAIXA:



                dist_obs, cubo, caixa = self.processar_sensores()







                d = caixa["dist"] if caixa else None



                a = caixa["ang"] if caixa else None







                v, w = self.inferencia_levar(d, a, dist_obs)







                if caixa and caixa["dist"] < 0.35:



                    self.base.move(0, 0, 0)



                    self.timer_estado = 0



                    self.estado_atual = Estado.SOLTANDO_CUBO



                    print("--> MUDANDO PARA SOLTAR")



                else:



                    self.base.move(v, 0, w)







            # ===============================



            # ESTADO: SOLTANDO CUBO



            # ===============================



            elif self.estado_atual == Estado.SOLTANDO_CUBO:



                self.timer_estado += 1







                if self.timer_estado == 20:



                    self.arm.move_to_drop()



                elif self.timer_estado == 60:



                    self.gripper.open()



                elif self.timer_estado == 100:



                    self.arm.move_to_compact()



                elif self.timer_estado > 120:



                    self.estado_atual = Estado.PROCURANDO_CUBO



                    print("--> REINICIANDO CICLO")











if __name__ == "__main__":



    c = YouBotController()



    c.run()