import cv2
import numpy as np

elevation_map = np.fromfile('./assets/elevation.data', dtype='uint8')
overrides_map = np.fromfile('./assets/overrides.data', dtype='uint8')

print(elevation_map.shape)
print(overrides_map.shape)

print(2048 * 2048)

elevation_map = elevation_map.reshape((2048, 2048))
overrides_map = overrides_map.reshape((2048, 2048))

print(elevation_map.shape)
print(overrides_map.shape)

print(elevation_map.dtype)
print(overrides_map.dtype)

print(np.max(elevation_map))
print(np.max(overrides_map))
print(overrides_map)

import matplotlib.pyplot as plt

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

plt.figure()
plt.imshow(elevation_map, cmap="viridis")
plt.colorbar()
plt.savefig('elevation_map.png')
plt.close()

plt.figure()
plt.imshow(overrides_map, cmap="viridis")
plt.colorbar()
plt.savefig('overrides_map.png')

plt.show()