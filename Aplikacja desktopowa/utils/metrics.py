import math

def calculate_metrics(history):
    """
    Calculates metrics from data history.
    
    Args:
        history (list): List of dicts with 'error' and 'distance' keys.
        
    Returns:
        dict: Calculated metrics (avgErrorPercent, stdDev, etc.)
    """
    if not history or len(history) < 2:
        return {
            "avgErrorPercent": 0.0,
            "stdDev": 0.0,
            "minDistance": 0.0,
            "maxDistance": 0.0
        }

    # Take last 25 samples for short-term stats (consistent with web app)
    recent_data = history[-25:]
    
    errors = [abs(d['error']) for d in recent_data]
    setpoints = [d['setpoint'] for d in recent_data]
    distances = [d['filtered'] for d in recent_data]
    
    # Calculate Mean
    avg_error = sum(errors) / len(errors)
    avg_setpoint = sum(setpoints) / len(setpoints) if len(setpoints) > 0 else 0
    
    avg_error_percent = (avg_error / avg_setpoint * 100) if avg_setpoint > 0 else 0
    
    # Calculate StdDev
    # std = sqrt(mean(abs(x - x.mean())**2))
    # Note: Using sample std dev (N-1) or population (N)? 
    # Numpy std is population by default, lets stick to population (N) or simple logic
    # Error variance
    mean_error_signed = sum([d['error'] for d in recent_data]) / len(recent_data)
    variance_sum = sum([pow(d['error'] - mean_error_signed, 2) for d in recent_data])
    variance = variance_sum / len(recent_data)
    std_dev = math.sqrt(variance)
    
    return {
        "avgErrorPercent": round(avg_error_percent, 1),
        "stdDev": round(std_dev, 1),
        "minDistance": min(distances),
        "maxDistance": max(distances)
    }
