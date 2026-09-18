"""Thin web UI for the robot arm. All comms live in motor_link.py."""

from contextlib import asynccontextmanager
from pathlib import Path
from typing import List

import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel, Field

from motor_link import AXIS_NUM, MotorLink, Parameters

INDEX_HTML = Path(__file__).parent / "templates" / "index.html"

link = MotorLink()


@asynccontextmanager
async def lifespan(app: FastAPI):
    link.start()
    yield


app = FastAPI(title="Robot Arm HMI", lifespan=lifespan)


# --- Models ------------------------------------------------------------
class SetpointsRequest(BaseModel):
    positions: List[int] = Field(..., min_length=AXIS_NUM, max_length=AXIS_NUM)


class ParametersRequest(BaseModel):
    kp: float
    ki: float
    kd: float
    output_min: int
    output_max: int


class ActivateRequest(BaseModel):
    enable: bool


# --- Routes ------------------------------------------------------------
@app.get("/", include_in_schema=False)
async def root() -> FileResponse:
    return FileResponse(INDEX_HTML)


@app.get("/favicon.ico", include_in_schema=False)
async def favicon() -> Response:
    return Response(status_code=204)


@app.get("/api/status")
async def get_status() -> dict:
    s = link.status()
    return {
        "connected": s.connected,
        "active": s.active,
        "client_ip": s.client_ip,
        "feedback_ok": s.feedback_ok,
        "positions": s.positions,
        "setpoints": s.setpoints,
        "parameters": vars(s.parameters),
    }


@app.post("/api/setpoints")
async def post_setpoints(req: SetpointsRequest) -> dict:
    link.set_setpoints(req.positions)
    return {"status": "ok"}


@app.post("/api/parameters")
async def post_parameters(req: ParametersRequest) -> dict:
    if not link.set_parameters(Parameters(**req.model_dump())):
        raise HTTPException(status_code=409, detail="No controller connected.")
    return {"status": "ok"}


@app.post("/api/activate")
async def post_activate(req: ActivateRequest) -> dict:
    if not link.activate(req.enable):
        raise HTTPException(status_code=409, detail="No controller connected.")
    return {"status": "ok"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)