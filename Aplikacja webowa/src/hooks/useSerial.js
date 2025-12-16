import { useCallback, useEffect, useRef, useState } from "react";

export const useSerial = () => {
    const [isConnected, setIsConnected] = useState(false);
    const [port, setPort] = useState(null);
    const [logs, setLogs] = useState([]);
    const [dataHistory, setDataHistory] = useState([]);
    const [latestData, setLatestData] = useState({
        distance: 0,
        filtered: 0,
        error: 0,
        control: 0,
        setpoint: 0,
    });

    const readerRef = useRef(null);
    const writerRef = useRef(null);
    const streamClosedRef = useRef(null);
    const streamsRef = useRef(null); // Keep references to the streams themselves
    const keepReadingRef = useRef(false);

    // Buffer for incoming chunks
    const bufferRef = useRef("");
    const MAX_HISTORY = 200;

    const log = useCallback((msg, type = "info") => {
        setLogs((prev) => [...prev.slice(-99), { time: new Date().toLocaleTimeString(), msg, type }]);
    }, []);

    const calculateCRC8 = (text) => {
        let crc = 0x00;
        for (let i = 0; i < text.length; i++) {
            let byte = text.charCodeAt(i);
            crc ^= byte;
            for (let j = 0; j < 8; j++) {
                if (crc & 0x80) {
                    crc = (crc << 1) ^ 0x07;
                } else {
                    crc <<= 1;
                }
                crc &= 0xff;
            }
        }
        return crc;
    };

    const processLine = useCallback(
        (line) => {
            // Expected format: D:123;A:100;F:120;E:10.5;C:AB
            // Or without CRC check if older format
            // Validation
            let valid = false;
            let payload = line;

            if (line.includes(";C:")) {
                const parts = line.split(";C:");
                if (parts.length === 2) {
                    const dataPart = parts[0];
                    const crcRecv = parseInt(parts[1], 16);
                    const crcCalc = calculateCRC8(dataPart);

                    if (crcCalc === crcRecv) {
                        valid = true;
                        payload = dataPart;
                    } else {
                        // log(`CRC Error: ${line}`, 'error');
                    }
                }
            } else {
                // Fallback (allow if desired, or strict mode)
                // valid = true;
            }

            if (valid) {
                const newData = { ...latestData };
                const segments = payload.split(";");
                let updated = false;

                segments.forEach((seg) => {
                    if (seg.startsWith("D:")) {
                        let val = parseFloat(seg.substring(2));
                        // Clamp 0-280 as per Python app
                        val = Math.max(0, Math.min(val, 280));
                        newData.distance = val;
                        updated = true;
                    } else if (seg.startsWith("F:")) {
                        newData.filtered = parseFloat(seg.substring(2));
                        updated = true;
                    } else if (seg.startsWith("E:")) {
                        newData.error = parseFloat(seg.substring(2));
                        updated = true;
                    } else if (seg.startsWith("A:")) {
                        newData.control = parseFloat(seg.substring(2));
                        updated = true;
                    }
                });

                if (updated) {
                    setLatestData((prev) => ({ ...prev, ...newData })); // Update current view

                    setDataHistory((prev) => {
                        const newHist = [...prev, { ...newData, timestamp: Date.now() }];
                        if (newHist.length > MAX_HISTORY) return newHist.slice(-MAX_HISTORY);
                        return newHist;
                    });
                }
            }
        },
        [latestData]
    );

    const connect = async () => {
        if (!navigator.serial) {
            alert("API Web Serial nie jest obsługiwane w tej przeglądarce.");
            return;
        }

        try {
            const selectedPort = await navigator.serial.requestPort();
            await selectedPort.open({ baudRate: 9600 }); // Default match Python app

            setPort(selectedPort);
            setIsConnected(true);
            log("Połączono z urządzeniem", "success");

            // Setup Read Loop
            const textDecoder = new TextDecoderStream();
            const readableStreamClosed = selectedPort.readable.pipeTo(textDecoder.writable);
            const reader = textDecoder.readable.getReader();

            const textEncoder = new TextEncoderStream();
            const writableStreamClosed = textEncoder.readable.pipeTo(selectedPort.writable);
            const writer = textEncoder.writable.getWriter();

            readerRef.current = reader;
            writerRef.current = writer;
            keepReadingRef.current = true;
            streamClosedRef.current = { readable: readableStreamClosed, writable: writableStreamClosed };
            streamsRef.current = { readable: textDecoder.readable, writable: textEncoder.writable };

            readLoop(reader);
        } catch (err) {
            log(`Błąd połączenia: ${err.message}`, "error");
            console.error(err);
        }
    };

    const readLoop = async (reader) => {
        try {
            while (keepReadingRef.current) {
                const { value, done } = await reader.read();
                if (done) break;
                if (value) {
                    bufferRef.current += value;
                    // Split by newline
                    const lines = bufferRef.current.split("\n");
                    // Process all complete lines
                    while (lines.length > 1) {
                        const line = lines.shift().trim();
                        if (line) processLine(line);
                    }
                    // Keep the last part (incomplete line)
                    bufferRef.current = lines[0];
                }
            }
        } catch (err) {
            log(`Utracono połączenie: ${err.message}`, "error");
            // Don't call disconnect() directly here to avoid race conditions.
            // The reader lock release below will allow cleanup.
            // We can trigger a state update that causes cleanup or just let user click disconnect.
            // But for auto-disconnect, we need to be careful.
            // Best: Call disconnect but ensure it handles "released" state.
            disconnect();
        } finally {
            try {
                reader.releaseLock();
            } catch (e) {
                console.error(e);
            }
        }
    };

    const disconnect = async () => {
        keepReadingRef.current = false;

        // 1. Close Reader
        if (readerRef.current) {
            try {
                await readerRef.current.cancel();
            } catch (e) {
                // Ignore "reader has been released" error
                if (!e.message.includes("released")) {
                    console.error("Error cancelling reader", e);
                }
            }
            readerRef.current = null;
        }

        // 1b. Ensure Readable Stream is definitely cancelled if reader was already released
        if (streamsRef.current?.readable) {
            try {
                const r = streamsRef.current.readable.getReader();
                await r.cancel();
                r.releaseLock();
            } catch (e) {
                // Ignore if already locked/closed
            }
        }

        // 2. Wait for Readable Stream Pipe to close
        if (streamClosedRef.current?.readable) {
            try {
                await streamClosedRef.current.readable.catch(() => {});
            } catch (e) {
                console.error(e);
            }
        }

        // 3. Close Writer
        if (writerRef.current) {
            try {
                await writerRef.current.close();
            } catch (e) {
                // If close fails (e.g. stream error), try abort
                try {
                    await writerRef.current.abort();
                } catch (e2) {}
            }
            writerRef.current = null;
        }

        // 4. Wait for Writable Stream Pipe to close
        if (streamClosedRef.current?.writable) {
            try {
                await streamClosedRef.current.writable.catch(() => {});
            } catch (e) {
                console.error(e);
            }
        }

        // 5. Close Port
        if (port) {
            try {
                await port.close();
            } catch (e) {
                console.error("Error closing port", e);
            }
        }

        setIsConnected(false);
        setPort(null);
        streamClosedRef.current = null;
        streamsRef.current = null;
        log("Rozłączono", "info");
    };

    const sendCommand = async (cmd) => {
        if (!writerRef.current) return;
        try {
            const crc = calculateCRC8(cmd);
            const msg = `${cmd};C:${crc.toString(16).toUpperCase().padStart(2, "0")}\n`;
            await writerRef.current.write(msg);
            log(`Wysłano: ${msg.trim()}`, "tx");
        } catch (err) {
            log(`Błąd wysyłania: ${err}`, "error");
        }
    };

    const sendSetpoint = (val) => {
        // Format: SET:150.0
        sendCommand(`SET:${val.toFixed(1)}`);
        setLatestData((prev) => ({ ...prev, setpoint: val })); // Optimistic update of local setpoint ref
    };

    const sendPid = (kp, ki, kd) => {
        // Format: PID:Kp;Ki;Kd
        sendCommand(`PID:${kp.toFixed(4)};${ki.toFixed(5)};${kd.toFixed(1)}`);
    };

    return {
        isConnected,
        connect,
        disconnect,
        logs,
        latestData,
        dataHistory,
        sendSetpoint,
        sendPid,
    };
};
