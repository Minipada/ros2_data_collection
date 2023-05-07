import os


def get_tz() -> str:
    """Get local timezone from localtime information."""
    return "/".join(os.path.realpath("/etc/localtime").split("/")[-2:])


def resample(df, default: str = "30S"):
    # calculate the length of the data in minutes
    if df.empty:
        return df
    data_length = (df["Date"].max() - df["Date"].min()).total_seconds() / 60
    if data_length == 0:
        return df
    df.set_index("Date", inplace=True)

    # Decide which resampling frequency to use based on the data length
    # > 24 hours -> Resample to daily data
    if data_length > 24 * 60:
        df_resampled = df.resample("D").mean()
    # > 2h -> resample to hourly data
    elif data_length > 2 * 60:
        df_resampled = df.resample("10T").mean()
    # Resample to 5 minutes data
    else:
        df_resampled = df.resample(default).mean()

    return df_resampled
