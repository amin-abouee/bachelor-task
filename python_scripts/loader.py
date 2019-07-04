import cv2
import numpy as np

elevation_map = np.fromfile('../assets/elevation.data', dtype='uint8')
overrides_map = np.fromfile('../assets/overrides.data', dtype='uint8')

print(elevation_map.shape)
print(overrides_map.shape)

# print(2048 * 2048)
# y = set(overrides_map)
# print('y: ', y)
import collections
res = collections.Counter(overrides_map)
print(res)

elevation_map = elevation_map.reshape((2048, 2048))
overrides_map = overrides_map.reshape((2048, 2048))

print(elevation_map.shape)
print(overrides_map.shape)

print(elevation_map.dtype)
print(overrides_map.dtype)

print(np.max(elevation_map))
print(np.max(overrides_map))
# print(overrides_map)

import matplotlib.pyplot as plt
import matplotlib

# fig = plt.figure(figsize=(4, 3), dpi=300)
# a = fig.add_subplot(1, 2, 1)
# imgplot = plt.imshow(elevation_map)
# a.set_title('Elevation Map')
# plt.colorbar(orientation='horizontal')
# a = fig.add_subplot(1, 2, 2)
# imgplot = plt.imshow(overrides_map)
# a.set_title('Overrides Map')
# plt.colorbar(orientation='horizontal')
# plt.show()
# plt.savefig('maps.png')

# https://matplotlib.org/3.1.0/tutorials/colors/colorbar_only.html
bounds = [0, 8, 32, 64, 88, 104, 120, 135]
bounds_val = [x for x in res.keys()]
print(bounds_val)
for x in res.keys():
    print("{}, {:08b}".format(x,x))

# cmap_10 = plt.cm.get_cmap('Dark2', 7) 
cmap_7 = matplotlib.colors.ListedColormap(["limegreen", "navy", "dodgerblue", "royalblue", "mediumblue", "orange", "firebrick"])
# cmap = matplotlib.colors.ListedColormap(["white", "black", "white", "white", "white", "white", "white"])
norm = matplotlib.colors.BoundaryNorm(bounds, cmap_7.N, clip=True)

plt.figure()
plt.imshow(elevation_map, cmap="viridis")
plt.colorbar()
plt.savefig('../maps/elevation_map.png')
plt.close()

# color_img = cv2.cvtColor(overrides_map, cv2.COLOR_GRAY2RGB)

plt.figure()
plt.imshow(overrides_map, cmap=cmap_7, norm=norm)
plt.colorbar(ticks=bounds_val)
plt.savefig('../maps/overrides_map.png')
plt.close()


cmap_2 = matplotlib.colors.ListedColormap(["steelblue", "greenyellow"])
valid_maps = (overrides_map == 0).astype(np.uint8)
plt.figure()
plt.imshow(valid_maps, cmap=cmap_2)
# plt.colorbar(ticks=bounds_val)
plt.savefig('../maps/valid_map.png')


plt.show()