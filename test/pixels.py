import math, pandas as pd
sensor_w_px=640
hfov_deg=60
h_fov_rad=math.radians(hfov_deg)
for d in [1,3,10,25]:
    width=2*d*math.tan(h_fov_rad/2)
    m_per_px=width/sensor_w_px
    deg_per_px=hfov_deg/sensor_w_px
    print(d, width, m_per_px, deg_per_px)

rows=[]
for d in [1,3,10,25]:
    width=2*d*math.tan(h_fov_rad/2)
    rows.append({'distance_m':d,'fov_width_m':width,'m_per_pixel':width/sensor_w_px,'deg_per_pixel':hfov_deg/sensor_w_px})
df=pd.DataFrame(rows)
df.to_csv('output/camera_pixel_subtense.csv', index=False)
df
