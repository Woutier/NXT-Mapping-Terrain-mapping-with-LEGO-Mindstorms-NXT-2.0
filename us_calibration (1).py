import time
import csv
import nxt.locator
import nxt.sensor
import nxt.sensor.generic

NUM_READINGS = 10       # readings per distance to average out noise

with nxt.locator.find() as brick:
    print("Connected:", brick.get_device_info()[0])

    us = brick.get_sensor(nxt.sensor.Port.S4, nxt.sensor.generic.Ultrasonic)

    results = []

    print("\nUltrasonic sensor calibration")
    print("Place an obstacle at a known distance in front of the sensor.")
    print("Type 'done' when finished.\n")

    while True:
        real = input("Real distance in cm (or 'done'): ").strip()

        if real.lower() == "done":
            break

        try:
            real_cm = float(real)
        except ValueError:
            print("Invalid number, try again.")
            continue

        # Take multiple readings and average
        readings = []
        for i in range(NUM_READINGS):
            readings.append(us.get_sample())
            time.sleep(0.1)

        avg_reading = sum(readings) / len(readings)

        ratio = real_cm / avg_reading if avg_reading > 0 else 0

        results.append({
            "real_cm": real_cm,
            "sensor_cm": round(avg_reading, 1),
            "ratio": round(ratio, 4)
        })

        print(f"  Sensor reads: {avg_reading:.1f} cm | Real: {real_cm} cm | Ratio: {ratio:.4f}\n")

    # Summary
    if len(results) > 0:
        print("\n" + "=" * 50)
        print("RESULTS")
        print("=" * 50)
        print(f"{'Real (cm)':>10} {'Sensor (cm)':>12} {'Ratio':>8}")
        print("-" * 50)

        for r in results:
            print(f"{r['real_cm']:>10.1f} {r['sensor_cm']:>12.1f} {r['ratio']:>8.4f}")

        avg_ratio = sum(r["ratio"] for r in results) / len(results)
        print("-" * 50)
        print(f"{'Average correction factor:':>24} {avg_ratio:.4f}")
        print(f"\nUse SENSOR_CORRECTION = {avg_ratio:.4f} in your mapping script.")

        with open("us_calibration.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Real (cm)", "Sensor (cm)", "Ratio"])
            for r in results:
                writer.writerow([r["real_cm"], r["sensor_cm"], r["ratio"]])
            writer.writerow([])
            writer.writerow(["Average correction factor", avg_ratio])
        print("Results saved to us_calibration.csv")
    else:
        print("No data collected.")
