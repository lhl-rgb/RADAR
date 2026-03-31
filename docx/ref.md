```cpp
class ClutterFactory {
public:
    static std::unique_ptr<IClutterGenerator> createClutterGenerator(const ClutterParams& params) {
        if (params.method == ClutterMethod::SIRP) {
            return std::make_unique<SIRPGenerator>();
        }

        switch (params.type) {
            case ClutterType::Rayleigh:
                return std::make_unique<RayleighGenerator>();
            case ClutterType::Weibull:
                return std::make_unique<WeibullGenerator>();
            case ClutterType::LogNormal:
                return std::make_unique<LogNormalGenerator>();
            case ClutterType::K_Dist:
                return std::make_unique<K_DistGenerator>();
            default:
                throw std::invalid_argument("Invalid clutter type");
        }
    }
};

```