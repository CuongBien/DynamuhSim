FROM python:3.8.13
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 MPLBACKEND=Agg MPLCONFIGDIR=/tmp/matplotlib
RUN python -m pip install --no-cache-dir pip==21.0 setuptools==65.5.0 wheel==0.38.0
COPY requirements.txt /tmp/round2-requirements.txt
RUN python -m pip install --no-cache-dir -r /tmp/round2-requirements.txt
RUN python -m pip install --no-cache-dir torch==1.13.1+cpu --extra-index-url https://download.pytorch.org/whl/cpu
RUN git clone https://github.com/sybrenstuvel/Python-RVO2.git /opt/Python-RVO2 && \
    git -C /opt/Python-RVO2 checkout c2c46ba8d59556aa10faf03479293236efea154d && \
    python -m pip install --no-cache-dir --no-build-isolation /opt/Python-RVO2
RUN git clone https://github.com/sepsamavi/safe-interactive-crowdnav.git /opt/CrowdSimPlus && \
    git -C /opt/CrowdSimPlus checkout d526a07c013c621fa1ad50894b6ba639f61063de && \
    python -m pip install --no-cache-dir --no-deps -e /opt/CrowdSimPlus
RUN python -c "import imageio_ffmpeg,os; os.symlink(imageio_ffmpeg.get_ffmpeg_exe(),'/usr/local/bin/ffmpeg')" && \
    python -m pip check
RUN python -m pip install --no-cache-dir gym==0.21.0 && python -m pip check
WORKDIR /work
CMD ["python", "--version"]
