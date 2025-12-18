import React, { useEffect, useRef } from "react";

export const Terminal = ({ logs }) => {
    const bodyRef = useRef(null);

    useEffect(() => {
        if (bodyRef.current) {
            bodyRef.current.scrollTop = bodyRef.current.scrollHeight;
        }
    }, [logs]);

    return (
        <div className="terminal-container">
            <div className="terminal-header">
                <span className="terminal-title">Logi Urządzenia</span>
                <button
                    className="btn-clear"
                    onClick={() => {
                        /* Handle clear optional */
                    }}
                >
                    Wyczyść
                </button>
            </div>
            <div className="terminal-body custom-scrollbar" ref={bodyRef}>
                {logs.length === 0 && <div className="text-[var(--text-muted)] italic p-2">Brak logów...</div>}
                {logs.map((log, i) => (
                    <div key={i} className="log-entry">
                        <span className="timestamp">[{log.time}]</span>
                        <span className={log.type === "error" ? "log-error" : log.type === "tx" ? "log-tx" : log.type === "success" ? "log-success" : ""}>{log.msg}</span>
                    </div>
                ))}
            </div>
        </div>
    );
};
