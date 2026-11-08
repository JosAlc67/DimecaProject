# Agarradera en L

Modelo CAD paramétrico (CadQuery) reconstruido a partir de las fotos de la pieza rota.

## Medidas usadas (mm)
| Lado | Largo | Espesor | Alto |
|---|---|---|---|
| A – placa con nervios blancos verticales | 47.0 | 3.3 | 36.0 |
| B – brazo con incruste negro | 52.9 | 15.8 | 29.5 |

Supuestos: largos medidos desde la esquina exterior; ambos lados apoyados en la misma base (Z=0).

Medidas **estimadas** (ajustables al inicio de `agarradera.py`): radio de la curva interior (8),
pared de la carcasa (1.6), profundidad del incruste (16), posición de los nervios (19 y 40),
labio superior / diente de clip, pines bajo los nervios (2).

## Archivos
- `agarradera.py` – modelo; `python3 agarradera.py` genera `salida/`
- `salida/agarradera_ensamble.step` – carcasa blanca + incruste negro (con colores)
- `salida/agarradera_carcasa.step|stl`, `salida/agarradera_incruste.step|stl`
- `salida/vistas.png` – vistas de referencia (`python3 render_vistas.py salida/vistas.png`)

Requiere: `pip install cadquery matplotlib`
