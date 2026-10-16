// Package discovery implements a discovery service that enumerates connected
// 3Dconnexion SpaceMouse devices and suggests input_controller configs for them.
package discovery

import (
	"context"
	"fmt"
	"sync"

	"github.com/pkg/errors"

	"go.viam.com/rdk/components/input"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/discovery"

	"github.com/viam-labs/spacemouse/spacemouse"
)

// Model is the discovery service model. Namespace matches the input_controller model.
var Model = resource.NewModel("viam-labs", "spacemouse", "discovery")

// Config is empty for now.
type Config struct {
	resource.TriviallyValidateConfig
}

func init() {
	resource.RegisterService(discovery.API, Model, resource.Registration[discovery.Service, *Config]{
		Constructor: newDiscovery,
	})
}

type service struct {
	resource.Named
	resource.AlwaysRebuild
	resource.TriviallyCloseable
	logger logging.Logger
	mu     sync.Mutex
}

func newDiscovery(
	_ context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (discovery.Service, error) {
	return &service{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}, nil
}

// DiscoverResources enumerates connected SpaceMouse devices and returns one
// suggested input_controller config per device.
func (s *service) DiscoverResources(_ context.Context, _ map[string]any) ([]resource.Config, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	devices, err := spacemouse.EnumerateConnected()
	if err != nil {
		return nil, errors.Wrap(err, "spacemouse enumerate failed")
	}
	if len(devices) == 0 {
		return nil, nil
	}

	configs := make([]resource.Config, 0, len(devices))
	for _, inst := range devices {
		attrs := map[string]any{
			"vendor_id":  fmt.Sprintf("%04x", inst.Device.VendorID),
			"product_id": fmt.Sprintf("%04x", inst.Device.ProductID),
		}
		if inst.SerialNumber != "" {
			attrs["serial"] = inst.SerialNumber
		}
		configs = append(configs, resource.Config{
			Name:       suggestedName(inst),
			API:        input.API,
			Model:      spacemouse.Model,
			Attributes: attrs,
		})
	}
	return configs, nil
}

func suggestedName(inst spacemouse.DeviceInstance) string {
	base := "spacemouse"
	if inst.SerialNumber == "" {
		return base
	}
	return base + "-" + inst.SerialNumber
}
