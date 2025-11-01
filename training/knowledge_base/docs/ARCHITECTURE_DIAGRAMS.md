# Architecture Diagrams

## Overview

This document provides visual representations of Jessy's architecture, including service topology, data flows, CI/CD pipelines, and deployment architecture.

---

## Service Architecture

### High-Level System Architecture

```mermaid
graph TB
    subgraph "External"
        User[User/Client]
        GH[GitHub Actions]
        Registry[Docker Registry]
    end
    
    subgraph "Docker Network (jessy-network)"
        API[jessy-api<br/>Go Service<br/>Port 3000]
        Core[jessy-core<br/>Rust Service<br/>Port 8080]
        
        subgraph "Shared Volumes"
            CargoCache[cargo-cache]
            TargetCache[target-cache]
            GoCache[go-cache]
            MmapData[mmap-data]
            TestResults[test-results]
        end
    end
    
    User -->|HTTP| API
    API -->|Internal API| Core
    Core -->|Read/Write| MmapData
    Core -->|Build Cache| CargoCache
    Core -->|Build Cache| TargetCache
    API -->|Module Cache| GoCache
    
    GH -->|Build & Push| Registry
    Registry -->|Pull Images| Core
    Registry -->|Pull Images| API
```

### Container Dependency Graph

```mermaid
graph LR
    subgraph "Development Mode"
        CoreDev[jessy-core<br/>development stage]
        APIDev[jessy-api<br/>development stage]
        Test[jessy-test<br/>test profile]
    end
    
    APIDev -->|depends_on<br/>service_healthy| CoreDev
    Test -->|depends_on| CoreDev
    Test -->|depends_on| APIDev
```


### Service Communication Flow

```mermaid
sequenceDiagram
    participant User
    participant API as jessy-api<br/>(Go)
    participant Core as jessy-core<br/>(Rust)
    participant MMAP as MMAP Files
    
    User->>API: HTTP Request
    API->>API: Validate Request
    API->>Core: Internal API Call
    Core->>MMAP: Read Dimensional Data
    MMAP-->>Core: Data
    Core->>Core: Process Query
    Core-->>API: Response
    API-->>User: HTTP Response
```

---

## Data Flow Architecture

### Dimensional Layer Data Flow

```mermaid
graph TD
    subgraph "Input"
        Query[User Query]
    end
    
    subgraph "Processing Pipeline"
        Parse[Query Parser]
        Navigate[Navigator]
        Scan[Dimension Scanner]
        Interfere[Interference Engine]
        Converge[Convergence Processor]
    end
    
    subgraph "Storage"
        Registry[Dimension Registry]
        MMAP[MMAP Regions]
        Pool[Memory Pool]
    end
    
    subgraph "Output"
        Result[Query Result]
    end
    
    Query --> Parse
    Parse --> Navigate
    Navigate --> Registry
    Registry --> Scan
    Scan --> MMAP
    MMAP --> Interfere
    Interfere --> Pool
    Pool --> Converge
    Converge --> Result
```


### Memory Management Flow

```mermaid
graph LR
    subgraph "Allocation"
        Request[Allocation Request]
        Manager[MMAP Manager]
        Region[MMAP Region]
    end
    
    subgraph "Storage"
        File[Memory-Mapped File]
        Kernel[OS Kernel]
        RAM[Physical RAM]
    end
    
    Request --> Manager
    Manager --> Region
    Region --> File
    File --> Kernel
    Kernel --> RAM
    
    RAM -.->|Page Fault| Kernel
    Kernel -.->|Load Page| File
```

---

## CI/CD Pipeline Architecture

### Complete CI/CD Flow

```mermaid
graph TB
    subgraph "Development"
        Dev[Developer]
        Local[Local Testing]
        Commit[Git Commit]
    end
    
    subgraph "GitHub Actions - CI"
        Trigger[Push/PR Trigger]
        Lint[Lint & Format]
        Test[Test Suite]
        Build[Build Images]
        Security[Security Scan]
        Bench[Benchmarks]
    end
    
    subgraph "GitHub Actions - CD"
        Deploy[Deploy Job]
        Staging[Staging Environment]
        Smoke[Smoke Tests]
        Prod[Production Environment]
    end
    
    subgraph "Artifacts"
        Registry[Docker Registry]
        Release[GitHub Release]
    end
    
    Dev --> Local
    Local --> Commit
    Commit --> Trigger
    
    Trigger --> Lint
    Trigger --> Security
    Lint --> Test
    Test --> Build
    Test --> Bench
    
    Build --> Registry
    Build --> Deploy
    Deploy --> Staging
    Staging --> Smoke
    Smoke --> Prod
    Prod --> Release
```


### CI Workflow Detail

```mermaid
stateDiagram-v2
    [*] --> Triggered
    Triggered --> Lint: Push/PR
    
    state Lint {
        [*] --> RustFmt
        RustFmt --> Clippy
        Clippy --> GoFmt
        GoFmt --> [*]
    }
    
    Lint --> Test
    
    state Test {
        [*] --> BuildTestImage
        BuildTestImage --> UnitTests
        UnitTests --> IntegrationTests
        IntegrationTests --> BDDTests
        BDDTests --> Coverage
        Coverage --> [*]
    }
    
    Test --> Build: main branch
    Test --> Benchmark: PR
    
    state Build {
        [*] --> BuildRust
        BuildRust --> BuildGo
        BuildGo --> PushRegistry
        PushRegistry --> [*]
    }
    
    state Benchmark {
        [*] --> RunBaseline
        RunBaseline --> RunCurrent
        RunCurrent --> Compare
        Compare --> Comment
        Comment --> [*]
    }
    
    Lint --> Security
    
    state Security {
        [*] --> TrivyScan
        TrivyScan --> CargoAudit
        CargoAudit --> ImageScan
        ImageScan --> [*]
    }
    
    Build --> [*]
    Benchmark --> [*]
    Security --> [*]
```


### Deployment Pipeline

```mermaid
graph TB
    subgraph "Trigger"
        MainPush[Push to main]
        Tag[Version Tag v*]
        Manual[Manual Dispatch]
    end
    
    subgraph "Staging Deployment"
        BuildStaging[Build Images]
        DeployStaging[Deploy to Staging]
        SmokeStaging[Smoke Tests]
    end
    
    subgraph "Production Deployment"
        VerifyTag[Verify Tag Format]
        BuildProd[Build & Tag Images]
        DeployProd[Deploy to Production]
        SmokeProd[Production Smoke Tests]
        CreateRelease[Create GitHub Release]
    end
    
    MainPush --> BuildStaging
    Manual --> BuildStaging
    BuildStaging --> DeployStaging
    DeployStaging --> SmokeStaging
    
    Tag --> VerifyTag
    Manual --> VerifyTag
    VerifyTag --> BuildProd
    BuildProd --> DeployProd
    DeployProd --> SmokeProd
    SmokeProd --> CreateRelease
```

---

## Network Topology

### Docker Network Architecture

```mermaid
graph TB
    subgraph "Host Machine"
        Host[Host OS<br/>macOS/Linux]
        
        subgraph "Docker Bridge Network: jessy-network"
            subgraph "jessy-core Container"
                CoreApp[Rust Application<br/>:8080]
                CoreHealth[Health Endpoint<br/>/health]
            end
            
            subgraph "jessy-api Container"
                APIApp[Go Application<br/>:3000]
                APIHealth[Health Endpoint<br/>/api/health]
            end
            
            Bridge[Bridge Interface<br/>172.18.0.0/16]
        end
        
        subgraph "Port Mappings"
            Port8080[0.0.0.0:8080]
            Port3000[0.0.0.0:3000]
        end
    end
    
    Host --> Port8080
    Host --> Port3000
    Port8080 --> CoreApp
    Port3000 --> APIApp
    APIApp -->|http://jessy-core:8080| CoreApp
    CoreApp -.->|DNS Resolution| Bridge
    APIApp -.->|DNS Resolution| Bridge
```


### Service Discovery

```mermaid
graph LR
    subgraph "Container: jessy-api"
        API[API Process]
        DNS1[DNS Resolver]
    end
    
    subgraph "Docker Network"
        Bridge[Bridge Network<br/>jessy-network]
        DNS[Docker DNS<br/>127.0.0.11]
    end
    
    subgraph "Container: jessy-core"
        Core[Core Process]
        DNS2[DNS Resolver]
    end
    
    API -->|Resolve 'jessy-core'| DNS1
    DNS1 -->|Query| DNS
    DNS -->|IP: 172.18.0.2| DNS1
    DNS1 -->|Connect| Core
    
    Core -->|Resolve 'jessy-api'| DNS2
    DNS2 -->|Query| DNS
    DNS -->|IP: 172.18.0.3| DNS2
    DNS2 -->|Connect| API
```

---

## Volume Architecture

### Volume Hierarchy

```mermaid
graph TB
    subgraph "Docker Volumes"
        subgraph "Build Caches"
            Cargo[cargo-cache<br/>~500MB<br/>Rust dependencies]
            Target[target-cache<br/>~2GB<br/>Build artifacts]
            Go[go-cache<br/>~100MB<br/>Go modules]
        end
        
        subgraph "Application Data"
            MMAP[mmap-data<br/>Variable<br/>Dimensional layers]
            Test[test-results<br/>~50MB<br/>Coverage reports]
        end
    end
    
    subgraph "Containers"
        CoreContainer[jessy-core]
        APIContainer[jessy-api]
        TestContainer[jessy-test]
    end
    
    Cargo --> CoreContainer
    Target --> CoreContainer
    Go --> APIContainer
    MMAP --> CoreContainer
    MMAP --> APIContainer
    Test --> TestContainer
    Cargo --> TestContainer
    Target --> TestContainer
```


### Volume Mount Strategy

```mermaid
graph LR
    subgraph "Host Filesystem"
        SrcDir[./src]
        APIDir[./api]
        CargoToml[./Cargo.toml]
    end
    
    subgraph "Container Filesystem"
        AppSrc[/app/src]
        AppAPI[/app/api]
        AppCargo[/app/Cargo.toml]
        
        subgraph "Named Volumes"
            CargoCacheVol[/usr/local/cargo/registry]
            TargetVol[/app/target]
        end
    end
    
    SrcDir -.->|bind mount| AppSrc
    APIDir -.->|bind mount| AppAPI
    CargoToml -.->|bind mount| AppCargo
    
    CargoCacheVol -.->|named volume| TargetVol
    
    style SrcDir fill:#e1f5ff
    style APIDir fill:#e1f5ff
    style CargoToml fill:#e1f5ff
    style CargoCacheVol fill:#fff4e1
    style TargetVol fill:#fff4e1
```

---

## Build Architecture

### Multi-Stage Docker Build

```mermaid
graph TB
    subgraph "Dockerfile.rust"
        subgraph "Stage 1: Builder"
            Base1[rust:1.83-slim]
            Deps1[Install build deps]
            CopyManifest[Copy Cargo.toml/lock]
            BuildDeps[Build dependencies]
            CopySrc[Copy source code]
            BuildApp[Build application]
        end
        
        subgraph "Stage 2: Development"
            Base2[rust:1.83-slim]
            DevTools[Install dev tools]
            InstallWatch[Install cargo-watch]
            CopyAll[Copy all source]
            RunWatch[CMD: cargo watch]
        end
        
        subgraph "Stage 3: Production"
            Base3[debian:bookworm-slim]
            Runtime[Install runtime deps]
            CopyBinary[Copy binary from builder]
            CreateUser[Create non-root user]
            SetCmd[CMD: ./jessy]
        end
    end
    
    Base1 --> Deps1
    Deps1 --> CopyManifest
    CopyManifest --> BuildDeps
    BuildDeps --> CopySrc
    CopySrc --> BuildApp
    
    Base2 --> DevTools
    DevTools --> InstallWatch
    InstallWatch --> CopyAll
    CopyAll --> RunWatch
    
    Base3 --> Runtime
    Runtime --> CopyBinary
    BuildApp -.->|COPY --from=builder| CopyBinary
    CopyBinary --> CreateUser
    CreateUser --> SetCmd
```


### Build Cache Strategy

```mermaid
graph TB
    subgraph "First Build"
        F1[Pull base image]
        F2[Install dependencies]
        F3[Copy Cargo.toml]
        F4[Build dependencies]
        F5[Copy source]
        F6[Build application]
    end
    
    subgraph "Cached Layers"
        C1[Base image layer]
        C2[Dependencies layer]
        C3[Manifest layer]
        C4[Dependency build layer]
    end
    
    subgraph "Subsequent Build"
        S1[Use cached base]
        S2[Use cached deps]
        S3[Use cached manifest]
        S4[Use cached dep build]
        S5[Copy new source]
        S6[Build application]
    end
    
    F1 --> C1
    F2 --> C2
    F3 --> C3
    F4 --> C4
    
    C1 --> S1
    C2 --> S2
    C3 --> S3
    C4 --> S4
    S4 --> S5
    S5 --> S6
    
    style C1 fill:#90EE90
    style C2 fill:#90EE90
    style C3 fill:#90EE90
    style C4 fill:#90EE90
```

---

## Testing Architecture

### Test Execution Flow

```mermaid
graph TB
    subgraph "Test Entry Points"
        MakeTest[make test]
        MakeUnit[make test-unit]
        MakeIntegration[make test-integration]
        MakeBDD[make test-bdd]
        MakeCoverage[make coverage]
    end
    
    subgraph "Test Container"
        TestImage[jessy-test image]
        
        subgraph "Test Suites"
            Unit[Unit Tests<br/>cargo test --lib]
            Integration[Integration Tests<br/>cargo test --test]
            BDD[BDD Tests<br/>cucumber]
            Bench[Benchmarks<br/>cargo bench]
        end
        
        subgraph "Coverage Tools"
            Tarpaulin[cargo-tarpaulin]
            Report[HTML Report]
        end
    end
    
    subgraph "Test Results"
        Results[test-results volume]
        Coverage[Coverage Report]
        Benchmarks[Benchmark Results]
    end
    
    MakeTest --> TestImage
    MakeUnit --> Unit
    MakeIntegration --> Integration
    MakeBDD --> BDD
    MakeCoverage --> Tarpaulin
    
    Unit --> Results
    Integration --> Results
    BDD --> Results
    Tarpaulin --> Coverage
    Bench --> Benchmarks
    
    Coverage --> Results
    Benchmarks --> Results
```


### Test Isolation Strategy

```mermaid
graph TB
    subgraph "Test Environment"
        subgraph "Isolated Network"
            TestCore[jessy-core<br/>test instance]
            TestAPI[jessy-api<br/>test instance]
        end
        
        subgraph "Test Data"
            TestDB[Test Database]
            TestMMAP[Test MMAP Files]
        end
        
        subgraph "Test Runner"
            Runner[Test Executor]
            Cleanup[Cleanup Handler]
        end
    end
    
    Runner --> TestCore
    Runner --> TestAPI
    TestCore --> TestMMAP
    TestAPI --> TestCore
    
    Runner --> Cleanup
    Cleanup -.->|After tests| TestDB
    Cleanup -.->|After tests| TestMMAP
    
    style TestDB fill:#FFE4E1
    style TestMMAP fill:#FFE4E1
```

---

## Health Check Architecture

### Health Check Flow

```mermaid
sequenceDiagram
    participant Docker
    participant Container
    participant HealthEndpoint
    participant Application
    
    loop Every 10 seconds
        Docker->>Container: Execute health check
        Container->>HealthEndpoint: curl /health
        HealthEndpoint->>Application: Check status
        Application-->>HealthEndpoint: Status
        HealthEndpoint-->>Container: HTTP 200/500
        Container-->>Docker: healthy/unhealthy
        
        alt Unhealthy
            Docker->>Docker: Increment failure count
            alt Failures >= 3
                Docker->>Container: Mark unhealthy
                Docker->>Container: Trigger restart
            end
        else Healthy
            Docker->>Docker: Reset failure count
        end
    end
```


### Service Startup Sequence

```mermaid
sequenceDiagram
    participant Compose as docker-compose
    participant Core as jessy-core
    participant API as jessy-api
    participant Health as Health Checks
    
    Compose->>Core: Start container
    activate Core
    Core->>Core: Initialize application
    Core->>Core: Load MMAP regions
    Core->>Core: Start HTTP server
    
    loop Health Check (start_period: 10s)
        Health->>Core: GET /health
        Core-->>Health: HTTP 200
    end
    
    Health->>Compose: jessy-core is healthy
    
    Compose->>API: Start container (depends_on)
    activate API
    API->>API: Initialize application
    API->>Core: Test connection
    Core-->>API: Connection OK
    API->>API: Start HTTP server
    
    loop Health Check
        Health->>API: GET /api/health
        API-->>Health: HTTP 200
    end
    
    Health->>Compose: jessy-api is healthy
    Compose->>Compose: All services ready
```

---

## Hot Reload Architecture

### Development Hot Reload Flow

```mermaid
graph TB
    subgraph "Developer Workflow"
        Dev[Developer]
        Editor[Code Editor]
        Save[Save File]
    end
    
    subgraph "File System"
        HostFS[Host Filesystem<br/>./src/lib.rs]
        Mount[Bind Mount]
        ContainerFS[Container Filesystem<br/>/app/src/lib.rs]
    end
    
    subgraph "Container: jessy-core"
        Watcher[cargo-watch]
        Compiler[cargo build]
        Process[Running Process]
    end
    
    Dev --> Editor
    Editor --> Save
    Save --> HostFS
    HostFS -.->|bind mount| Mount
    Mount -.->|sync| ContainerFS
    
    ContainerFS -->|inotify event| Watcher
    Watcher -->|trigger| Compiler
    Compiler -->|kill old| Process
    Compiler -->|start new| Process
    
    style Save fill:#FFD700
    style Watcher fill:#90EE90
```


### Hot Reload Timing

```mermaid
gantt
    title Hot Reload Timeline
    dateFormat ss
    axisFormat %S
    
    section File Change
    Developer saves file :done, save, 00, 1s
    
    section Detection
    File system event :done, detect, 01, 1s
    cargo-watch detects :done, watch, 02, 1s
    
    section Compilation
    Incremental compile :active, compile, 03, 3s
    
    section Restart
    Kill old process :crit, kill, 06, 1s
    Start new process :done, start, 07, 2s
    
    section Ready
    Service ready :milestone, ready, 09, 0s
```

---

## Deployment Architecture

### Production Deployment

```mermaid
graph TB
    subgraph "GitHub"
        Tag[Version Tag<br/>v1.2.3]
        Actions[GitHub Actions]
        Registry[GitHub Container Registry]
    end
    
    subgraph "Build Pipeline"
        Build[Build Multi-arch Images]
        Test[Run Test Suite]
        Scan[Security Scan]
        Push[Push to Registry]
    end
    
    subgraph "Production Environment"
        LB[Load Balancer]
        
        subgraph "Instance 1"
            Core1[jessy-core]
            API1[jessy-api]
        end
        
        subgraph "Instance 2"
            Core2[jessy-core]
            API2[jessy-api]
        end
        
        Storage[Persistent Storage]
    end
    
    Tag --> Actions
    Actions --> Build
    Build --> Test
    Test --> Scan
    Scan --> Push
    Push --> Registry
    
    Registry --> Core1
    Registry --> API1
    Registry --> Core2
    Registry --> API2
    
    LB --> API1
    LB --> API2
    
    API1 --> Core1
    API2 --> Core2
    
    Core1 --> Storage
    Core2 --> Storage
```


### Blue-Green Deployment Strategy

```mermaid
graph TB
    subgraph "Load Balancer"
        LB[Traffic Router]
    end
    
    subgraph "Blue Environment (Current)"
        BlueAPI[jessy-api v1.2.2]
        BlueCore[jessy-core v1.2.2]
    end
    
    subgraph "Green Environment (New)"
        GreenAPI[jessy-api v1.2.3]
        GreenCore[jessy-core v1.2.3]
    end
    
    subgraph "Shared Resources"
        DB[(Database)]
        Storage[(Storage)]
    end
    
    LB -->|100% traffic| BlueAPI
    LB -.->|0% traffic| GreenAPI
    
    BlueAPI --> BlueCore
    GreenAPI --> GreenCore
    
    BlueCore --> DB
    GreenCore --> DB
    BlueCore --> Storage
    GreenCore --> Storage
    
    style BlueAPI fill:#87CEEB
    style BlueCore fill:#87CEEB
    style GreenAPI fill:#90EE90
    style GreenCore fill:#90EE90
```

---

## Security Architecture

### Security Layers

```mermaid
graph TB
    subgraph "External Layer"
        Internet[Internet]
        Firewall[Firewall]
    end
    
    subgraph "Network Layer"
        LB[Load Balancer<br/>TLS Termination]
        WAF[Web Application Firewall]
    end
    
    subgraph "Application Layer"
        API[jessy-api<br/>Input Validation]
        Auth[Authentication]
        RateLimit[Rate Limiting]
    end
    
    subgraph "Service Layer"
        Core[jessy-core<br/>Business Logic]
        Validation[Data Validation]
    end
    
    subgraph "Data Layer"
        MMAP[MMAP Files<br/>Read-only]
        Secrets[Secrets Manager]
    end
    
    Internet --> Firewall
    Firewall --> LB
    LB --> WAF
    WAF --> API
    API --> Auth
    Auth --> RateLimit
    RateLimit --> Core
    Core --> Validation
    Validation --> MMAP
    API -.->|fetch| Secrets
    Core -.->|fetch| Secrets
```


### Container Security Model

```mermaid
graph TB
    subgraph "Host OS"
        Kernel[Linux Kernel]
        
        subgraph "Namespaces"
            PID[PID Namespace]
            NET[Network Namespace]
            MNT[Mount Namespace]
            USER[User Namespace]
        end
        
        subgraph "Control Groups"
            CPU[CPU Limits]
            MEM[Memory Limits]
            IO[I/O Limits]
        end
    end
    
    subgraph "Container"
        App[Application<br/>Non-root user]
        FS[Read-only Filesystem]
        Caps[Dropped Capabilities]
    end
    
    Kernel --> PID
    Kernel --> NET
    Kernel --> MNT
    Kernel --> USER
    
    Kernel --> CPU
    Kernel --> MEM
    Kernel --> IO
    
    PID --> App
    NET --> App
    MNT --> FS
    USER --> App
    
    CPU --> App
    MEM --> App
    IO --> App
    
    App --> Caps
    
    style App fill:#90EE90
    style FS fill:#FFD700
    style Caps fill:#FFD700
```

---

## Monitoring Architecture (Future)

### Observability Stack

```mermaid
graph TB
    subgraph "Application Layer"
        Core[jessy-core]
        API[jessy-api]
    end
    
    subgraph "Metrics Collection"
        Prometheus[Prometheus]
        Exporter1[Rust Metrics Exporter]
        Exporter2[Go Metrics Exporter]
    end
    
    subgraph "Visualization"
        Grafana[Grafana]
        Dashboard1[System Dashboard]
        Dashboard2[Application Dashboard]
    end
    
    subgraph "Tracing"
        Jaeger[Jaeger]
        Collector[Trace Collector]
    end
    
    subgraph "Logging"
        Loki[Loki]
        Aggregator[Log Aggregator]
    end
    
    Core --> Exporter1
    API --> Exporter2
    Exporter1 --> Prometheus
    Exporter2 --> Prometheus
    
    Prometheus --> Grafana
    Grafana --> Dashboard1
    Grafana --> Dashboard2
    
    Core --> Collector
    API --> Collector
    Collector --> Jaeger
    
    Core --> Aggregator
    API --> Aggregator
    Aggregator --> Loki
    Loki --> Grafana
```

---

## Related Documentation

- [Docker Setup](DOCKER_SETUP.md)
- [CI/CD Pipeline](CI_CD.md)
- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)
- [Troubleshooting Guide](TROUBLESHOOTING.md)

---

*"A picture is worth a thousand words. A diagram is worth a thousand lines of code. 📐"*
