import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Load the data
file_path = 'Tests/Scenario 4/Test3/aggregated_data.csv'
df = pd.read_csv(file_path)

# Set marker column as a variable
compare_column = 'range'

# Ensure that timestamp is in datetime format
df['timestamp'] = pd.to_datetime(df['timestamp'])

# Normalize the time for easier slider control
df['time_float'] = (df['timestamp'] - df['timestamp'].min()).dt.total_seconds()

# Create the figure and axis objects
fig, ax = plt.subplots(figsize=(12, 6))
plt.subplots_adjust(left=0.1, bottom=0.25)

# Plot initial data using color-blind safe colors
ground_truth_line, = ax.plot([], [], label='Ground Truth', marker='o', linestyle='-', color='#ffff33')  # blue
compare_line, = ax.plot([], [], label=compare_column.capitalize(), marker='x', linestyle='--', color='#377eb8')  # orange

# Set up the sliders for time range selection
ax_time_start = plt.axes([0.1, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
ax_time_end = plt.axes([0.1, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')

slider_time_start = Slider(ax_time_start, 'Start Time', df['time_float'].min(), df['time_float'].max(), valinit=df['time_float'].min())
slider_time_end = Slider(ax_time_end, 'End Time', df['time_float'].min(), df['time_float'].max(), valinit=df['time_float'].max())

# Update function for sliders
def update(val):
    start_time = pd.to_datetime(df['timestamp'].min()) + pd.to_timedelta(slider_time_start.val, unit='s')
    end_time = pd.to_datetime(df['timestamp'].min()) + pd.to_timedelta(slider_time_end.val, unit='s')
    
    df_filtered = df[(df['timestamp'] >= start_time) & (df['timestamp'] <= end_time)]
    
    ground_truth_line.set_data(df_filtered['timestamp'], df_filtered['ground_truth'])
    compare_line.set_data(df_filtered['timestamp'], df_filtered[compare_column])
    
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw_idle()

# Attach the update function to the sliders
slider_time_start.on_changed(update)
slider_time_end.on_changed(update)

# Initial plot setup
ax.set_title('Ground Truth and ' + compare_column.capitalize() + ' over Time')
ax.set_xlabel('Timestamp')
ax.set_ylabel('Articulation Angle (radians)')
ax.legend()
ax.grid(True)

# Plot the initial data
update(None)

plt.show()
