import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

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

# Create an inset axis
axins = inset_axes(ax, width="30%", height="30%", loc=2)  # Upper left corner

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
    
    # Convert pandas Series to NumPy arrays
    timestamps = df_filtered['timestamp'].to_numpy()
    ground_truth_values = df_filtered['ground_truth'].to_numpy()
    compare_values = df_filtered[compare_column].to_numpy()
    
    # Update main plot
    ax.clear()
    ax.plot(timestamps, ground_truth_values, label='Ground Truth', marker='o', linestyle='-', color='#ffff33')
    ax.plot(timestamps, compare_values, label=compare_column.capitalize(), marker='x', linestyle='--', color='#377eb8')
    ax.set_title('Ground Truth and ' + compare_column.capitalize() + ' over Time')
    ax.set_xlabel('Timestamp')
    ax.set_ylabel('Articulation Angle (radians)')
    ax.legend()
    ax.grid(True)

    # Update inset plot
    axins.clear()
    axins.plot(timestamps, ground_truth_values, color='#ffff33')
    axins.plot(timestamps, compare_values, color='#377eb8')
    axins.set_xlim(timestamps[0], timestamps[-1])  # Full range visible in the inset
    axins.set_ylim(min(compare_values), max(compare_values))  # Auto-adjust y-axis
    axins.grid(True)
    
    fig.canvas.draw_idle()

# Attach the update function to the sliders
slider_time_start.on_changed(update)
slider_time_end.on_changed(update)

# Initial data update
update(None)

plt.show()
