FROM dynamuhsim-crowdsimplus:round2
COPY requirements.txt /tmp/round3-requirements.txt
RUN python -m pip install --no-cache-dir -r /tmp/round3-requirements.txt
RUN python -m pip install --no-cache-dir torchvision==0.14.1+cpu --extra-index-url https://download.pytorch.org/whl/cpu
RUN git clone https://github.com/ChanganVR/socialforce.git /opt/ComplexityNav-socialforce && \
    git -C /opt/ComplexityNav-socialforce checkout 9fdbdf624a1ae8baef850d7c2cc9859198f3c97b && \
    python -m pip install --no-cache-dir --no-deps /opt/ComplexityNav-socialforce
RUN git clone https://github.com/fluentrobotics/ComplexityNav.git /opt/ComplexityNav && \
    git -C /opt/ComplexityNav checkout 36acac12a940a20dceb7b4e1d36acc4c7a4e344a && \
    python -m pip install --no-cache-dir --no-deps -e /opt/ComplexityNav
RUN mkdir -p /home/socnav/arstr && ln -s /opt/ComplexityNav /home/socnav/arstr/RelationalGraphLearning
ENV PYTHONPATH=/opt/ComplexityNav
WORKDIR /work
CMD ["python", "--version"]
