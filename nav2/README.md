>Note: Quick documentation, will be expanded later

- Start NAV2 container:
```bash
docker compose up
```

- Start Nav2 for use with a simulation or Rosbag:
```bash
EDU_USE_SIM_TIME=true docker compose up
```

- Start Nav2 with keepout-filter (requires filter-map-server started by SLAM-container):
```bash
USE_KEEPOUT=true docker compose up
```

- Stop Container
```bash
docker compose down
```