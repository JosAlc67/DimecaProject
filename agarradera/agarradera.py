"""
Agarradera en L - modelo parametrico (CadQuery).

Reconstruido a partir de fotos de la pieza rota + medidas tomadas a mano.
Unidades: mm.

Ejes:
  X -> a lo largo del lado A (placa delgada con nervios verticales blancos)
  Y -> a lo largo del lado B (brazo grueso con incruste negro)
  Z -> altura

Ejecutar:  python3 agarradera.py   -> genera STEP y STL en ./salida/
"""
import os
import cadquery as cq

# ---------------- MEDIDAS DADAS ----------------
A_LARGO, A_ESP, A_ALTO = 47.0, 3.3, 36.0     # lado A (lineas blancas verticales)
B_LARGO, B_ESP, B_ALTO = 52.9, 15.8, 29.5    # lado B (incruste negro)

# ---------------- MEDIDAS ESTIMADAS (ajustar) ----------------
R_INT = 8.0          # radio de la transicion curva interior entre A y B
R_EXT = 2.0          # redondeo de la esquina exterior
PARED = 1.6          # pared de la carcasa blanca alrededor del incruste
INC_PROF = 16.0      # profundidad del alojamiento del incruste negro
ALETA_ESP = 1.0      # aleta blanca que entra en el incruste (linea blanca en la punta)
ALETA_LARGO = 8.0
ALETA_Z = B_ALTO * 0.70
NERV_X = (19.0, 40.0)  # posicion de los nervios verticales (desde el extremo libre de A)
NERV_ANCHO, NERV_SAL, PIN = 1.2, 1.5, 2.0   # ancho, saliente, pin bajo la base
LABIO_ALTO, LABIO_SAL = 5.0, 2.0            # pestaña superior en cara exterior de A
TAB_ANCHO, TAB_ALTO = 7.0, 1.5              # diente de clip sobre el labio

bx0 = A_LARGO - B_ESP   # inicio en X del brazo B


def carcasa():
    # Lado A: placa delgada
    a = cq.Workplane("XY").box(A_LARGO, A_ESP, A_ALTO, centered=False)
    # Lado B: brazo grueso
    b = (cq.Workplane("XY").box(B_ESP, B_LARGO, B_ALTO, centered=False)
         .translate((bx0, 0, 0)))
    cuerpo = a.union(b)

    # Esquina exterior redondeada
    cuerpo = cuerpo.edges("|Z").edges(cq.selectors.NearestToPointSelector((A_LARGO, 0, 10))).fillet(R_EXT)

    # Transicion curva interior (filete concavo entre A y B)
    filete = (cq.Workplane("XY").box(R_INT, R_INT, B_ALTO, centered=False)
              .translate((bx0 - R_INT, A_ESP, 0))
              .cut(cq.Workplane("XY").circle(R_INT).extrude(B_ALTO)
                   .translate((bx0 - R_INT, A_ESP + R_INT, 0))))
    cuerpo = cuerpo.union(filete)

    # Alojamiento del incruste negro en la punta de B (abierto por la punta)
    bolsillo = (cq.Workplane("XY")
                .box(B_ESP - 2 * PARED, INC_PROF + 1, B_ALTO - 2 * PARED, centered=False)
                .translate((bx0 + PARED, B_LARGO - INC_PROF, PARED)))
    cuerpo = cuerpo.cut(bolsillo)

    # Aleta blanca interior (se ve como linea blanca en la punta)
    aleta = (cq.Workplane("XY").box(ALETA_LARGO, INC_PROF, ALETA_ESP, centered=False)
             .translate((A_LARGO - PARED - ALETA_LARGO, B_LARGO - INC_PROF, ALETA_Z)))
    cuerpo = cuerpo.union(aleta)

    # Nervios verticales en la cara exterior de A (con pin bajo la base)
    for x in NERV_X:
        n = (cq.Workplane("XY").box(NERV_ANCHO, NERV_SAL, A_ALTO - LABIO_ALTO + PIN, centered=False)
             .translate((x - NERV_ANCHO / 2, -NERV_SAL, -PIN)))
        cuerpo = cuerpo.union(n)

    # Labio superior + diente de clip en cara exterior de A
    labio = (cq.Workplane("XY").box(A_LARGO - 6, LABIO_SAL, LABIO_ALTO, centered=False)
             .translate((3, -LABIO_SAL, A_ALTO - LABIO_ALTO)))
    tab = (cq.Workplane("XY").box(TAB_ANCHO, LABIO_SAL, TAB_ALTO, centered=False)
           .translate((A_LARGO / 2 - TAB_ANCHO / 2 + 4, -LABIO_SAL, A_ALTO)))
    cuerpo = cuerpo.union(labio).union(tab)
    return cuerpo


def incruste():
    holg = 0.1
    ins = (cq.Workplane("XY")
           .box(B_ESP - 2 * PARED - 2 * holg, INC_PROF, B_ALTO - 2 * PARED - 2 * holg, centered=False)
           .translate((bx0 + PARED + holg, B_LARGO - INC_PROF, PARED + holg)))
    ranura = (cq.Workplane("XY").box(ALETA_LARGO + holg, INC_PROF, ALETA_ESP + 2 * holg, centered=False)
              .translate((A_LARGO - PARED - ALETA_LARGO - holg, B_LARGO - INC_PROF, ALETA_Z - holg)))
    ins = ins.edges("|Y").fillet(0.8)
    return ins.cut(ranura)


if __name__ == "__main__":
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "salida")
    os.makedirs(out, exist_ok=True)
    c, i = carcasa(), incruste()
    cq.exporters.export(c, os.path.join(out, "agarradera_carcasa.step"))
    cq.exporters.export(c, os.path.join(out, "agarradera_carcasa.stl"), tolerance=0.02)
    cq.exporters.export(i, os.path.join(out, "agarradera_incruste.step"))
    cq.exporters.export(i, os.path.join(out, "agarradera_incruste.stl"), tolerance=0.02)
    asm = (cq.Assembly(name="agarradera")
           .add(c, name="carcasa", color=cq.Color(0.93, 0.89, 0.78))
           .add(i, name="incruste", color=cq.Color(0.08, 0.08, 0.08)))
    asm.save(os.path.join(out, "agarradera_ensamble.step"))
    bb = c.val().BoundingBox()
    print("Carcasa bbox: %.1f x %.1f x %.1f mm" % (bb.xlen, bb.ylen, bb.zlen))
